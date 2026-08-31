/* Serial Bridge — UART <-> FSK RF tunnel for UV-K5 V3 (PY32F071)
 *
 * Same 72-byte air frame as V1 (DP32G030) so the two radios interoperate:
 *   [0]      = 0xABCD
 *   [1]      = (seq << 8) | len   (len = 1..56)
 *   [2..29]  = payload (up to 56 bytes)
 *   [30..33] = padding (0)
 *   [34]     = CRC16-CCITT over bytes of words [1..33] (66 bytes)
 *   [35]     = 0xDCBA
 *
 * UART RX uses PY32 DMA remaining-count instead of DP32 DMA_CH0->ST.
 *
 * XOR mode: FSK UART at idle (speaker off); analog FM during a QSO (FSK off).
 */

#ifdef ENABLE_SERIAL_BRIDGE

#include "app/serial_bridge.h"

#include <string.h>

#include "app/app.h"
#include "app/generic.h"
#include "audio.h"
#include "driver/bk4819.h"
#include "driver/crc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "frequencies.h"
#include "functions.h"
#include "misc.h"
#include "py32f071_ll_dma.h"
#include "radio.h"
#include "settings.h"
#include "ui/inputbox.h"
#include "ui/status.h"
#include "ui/ui.h"

#define SB_SYNC_HEAD   0xABCDu
#define SB_SYNC_TAIL   0xDCBAu
#define SB_UART_BUF    128u
#define SB_IDLE_TICKS  3u   /* ~30 ms at 10 ms slice before flush */
#define SB_TX_GAP      20u  /* ~200 ms mute after TX: drop self-RX + USB EMI */
#define SB_BUSY_WAIT   20u  /* max ~200 ms wait if channel busy */
#define SB_UART_DMA_CH LL_DMA_CHANNEL_2

static uint8_t  s_uart_rx[SB_UART_BUF];
static uint8_t  s_uart_len;
static uint16_t s_uart_dma_idx;
static uint8_t  s_idle_ticks;
static uint8_t  s_tx_gap;
static uint8_t  s_busy_wait;
static uint8_t  s_tx_seq;
static uint8_t  s_last_rx_seq;
static bool     s_have_rx_seq;
static bool     s_active;
static bool     s_pending_rearm;
static uint8_t  s_saved_dual_watch;
static uint8_t  s_saved_cross_band;

uint16_t gSerialBridgeFSKBuffer[36];
uint16_t gSerialBridgeTxPackets;
uint16_t gSerialBridgeRxPackets;
uint16_t gSerialBridgeRxErrors;
uint16_t gSerialBridgeTxBytes;
uint16_t gSerialBridgeRxBytes;
bool     gSerialBridgeBusyTx;

static uint16_t SB_UartDmaWriteIndex(void)
{
	uint32_t remaining;

	if (!LL_DMA_IsEnabledChannel(DMA1, SB_UART_DMA_CH))
		return 0;

	remaining = LL_DMA_GetDataLength(DMA1, SB_UART_DMA_CH);
	if (remaining > sizeof(UART_DMA_Buffer))
		return 0;

	return (uint16_t)(sizeof(UART_DMA_Buffer) - remaining);
}

static void SB_SetupFSK(void)
{
	/* FSK modem only. Do not enable Tone-2 (REG_70): it mutes analog FM RX. */
	BK4819_WriteRegister(BK4819_REG_72, 0x3065);
	BK4819_WriteRegister(BK4819_REG_58, 0x00C1);
	BK4819_WriteRegister(BK4819_REG_5C, 0x5665);
	BK4819_WriteRegister(BK4819_REG_5D, 0x4700);
}

static bool SB_AnalogRxActive(void)
{
	return gCurrentFunction == FUNCTION_RECEIVE
	    || gCurrentFunction == FUNCTION_INCOMING
	    || gCurrentFunction == FUNCTION_MONITOR
	    || gMonitor;
}

static void SB_SpeakerOff(void)
{
	AUDIO_AudioPathOff();
	gEnableSpeaker = false;
}

static void SB_RestoreAnalogRx(void)
{
	/* Idle FSK overlay: analog demod stays configured, speaker stays off. */
	BK4819_WriteRegister(BK4819_REG_70, 0x0000);
	RADIO_SetModulation(gRxVfo->Modulation);
	BK4819_WriteRegister(BK4819_REG_48,
		(11u << 12) |
		( 0u << 10) |
		(gEeprom.VOLUME_GAIN << 4) |
		(gEeprom.DAC_GAIN << 0));
	SB_SpeakerOff();
}

static void SB_DiscardUartNoise(void)
{
	/* EMI during FSK TX shows up as 0xFF on the programming cable UART. */
	s_uart_dma_idx = SB_UartDmaWriteIndex();
}

static void SB_DrainUart(void)
{
	const uint16_t dma_len = SB_UartDmaWriteIndex();

	while (s_uart_dma_idx != dma_len && s_uart_len < SB_UART_BUF) {
		s_uart_rx[s_uart_len++] = UART_DMA_Buffer[s_uart_dma_idx];
		s_uart_dma_idx = (uint16_t)((s_uart_dma_idx + 1u) % sizeof(UART_DMA_Buffer));
		s_idle_ticks = 0;
	}
}

static bool SB_AnalogBusy(void)
{
	return gPttIsPressed || gCurrentFunction == FUNCTION_TRANSMIT;
}

static void SB_ApplyFrequency(uint32_t Frequency)
{
	const uint8_t Vfo = gEeprom.TX_VFO;
	const FREQUENCY_Band_t band = FREQUENCY_GetBand(Frequency);
	uint16_t step = gTxVfo->StepFrequency;

	if (step == 0 || step == 833)
		step = 1250;

	Frequency = FREQUENCY_RoundToStep(Frequency, step);

	gTxVfo->Band = band;
	gTxVfo->CHANNEL_SAVE = (uint16_t)(FREQ_CHANNEL_FIRST + band);
	gEeprom.ScreenChannel[Vfo] = gTxVfo->CHANNEL_SAVE;
	gEeprom.FreqChannel[Vfo]   = gTxVfo->CHANNEL_SAVE;
	gTxVfo->freq_config_RX.Frequency = Frequency;
	gTxVfo->freq_config_TX.Frequency = Frequency;
	gTxVfo->TX_OFFSET_FREQUENCY = 0;
	gTxVfo->TX_OFFSET_FREQUENCY_DIRECTION = TX_OFFSET_FREQUENCY_DIRECTION_OFF;
	gTxVfo->FrequencyReverse = false;
	gTxVfo->pRX = &gTxVfo->freq_config_RX;
	gTxVfo->pTX = &gTxVfo->freq_config_TX;

	SETTINGS_SaveVfoIndices();
	SETTINGS_SaveChannel(gTxVfo->CHANNEL_SAVE, Vfo, gTxVfo, 1);

	RADIO_SelectVfos();
	gCurrentVfo = gRxVfo;
	RADIO_ConfigureSquelchAndOutputPower(gRxVfo);
	RADIO_SetupRegisters(true);
	gRequestDisplayScreen = DISPLAY_SERIAL_BRIDGE;
	gUpdateDisplay = true;
}

void SERIAL_BRIDGE_ReArm(void)
{
	uint16_t mask;

	if (!s_active)
		return;

	if (gPttIsPressed || gCurrentFunction == FUNCTION_TRANSMIT)
		return;

	if (SB_AnalogRxActive()
	    || g_SquelchLost
	    || gEndOfRxDetectedMaybe
	    || gTailNoteEliminationCountdown_10ms > 0
	    || s_tx_gap > 0)
		return;

	s_pending_rearm = false;

	gFSKWriteIndex = 0;
	SB_SetupFSK();

	/* Enable FSK RX without BK4819_ResetFSK()/Idle() so analog FM stays up. */
	BK4819_WriteRegister(BK4819_REG_59, 0x4068);
	BK4819_WriteRegister(BK4819_REG_59, 0x3068);

	mask = BK4819_ReadRegister(BK4819_REG_3F);
	mask |= (uint16_t)(
		BK4819_REG_3F_FSK_RX_FINISHED |
		BK4819_REG_3F_FSK_FIFO_ALMOST_FULL |
		BK4819_REG_3F_SQUELCH_FOUND |
		BK4819_REG_3F_SQUELCH_LOST |
		BK4819_REG_3F_CxCSS_TAIL);
	BK4819_WriteRegister(BK4819_REG_3F, mask);
	SB_RestoreAnalogRx();
}

void SERIAL_BRIDGE_LeaveFsk(void)
{
	if (!s_active)
		return;

	gSerialBridgeBusyTx = false;
	gFSKWriteIndex = 0;
	s_pending_rearm = true;

	/* FSK (REG_58) and Tone-2 (REG_70) steal analog MIC deviation. */
	BK4819_WriteRegister(BK4819_REG_58, 0x0000);
	BK4819_WriteRegister(BK4819_REG_70, 0x0000);
	BK4819_WriteRegister(BK4819_REG_59, 0x0068);
	BK4819_WriteRegister(BK4819_REG_7D, 0xE940 | (gEeprom.MIC_SENSITIVITY_TUNING & 0x1f));
}

void SERIAL_BRIDGE_EnterAnalogRx(void)
{
	if (!s_active)
		return;

	s_pending_rearm = true;
	gFSKWriteIndex = 0;

	/* Disable FSK modem only. Do not write REG_59=0x0068: that mutes AF on V1. */
	BK4819_WriteRegister(BK4819_REG_58, 0x0000);
	BK4819_WriteRegister(BK4819_REG_70, 0x0000);
	BK4819_RX_TurnOn();
}

void SERIAL_BRIDGE_HoldAfterTx(void)
{
	if (!s_active)
		return;

	s_tx_gap = SB_TX_GAP;
	s_pending_rearm = true;
}

static void SB_SendPayload(const uint8_t *data, uint8_t len)
{
	uint8_t *payload;

	if (len == 0 || len > SERIAL_BRIDGE_PAYLOAD_MAX)
		return;

	memset(gSerialBridgeFSKBuffer, 0, sizeof(gSerialBridgeFSKBuffer));
	gSerialBridgeFSKBuffer[0] = SB_SYNC_HEAD;
	gSerialBridgeFSKBuffer[1] = ((uint16_t)s_tx_seq << 8) | len;
	payload = (uint8_t *)&gSerialBridgeFSKBuffer[2];
	memcpy(payload, data, len);
	gSerialBridgeFSKBuffer[34] = CRC_Calculate(&gSerialBridgeFSKBuffer[1], 2 + 64);
	gSerialBridgeFSKBuffer[35] = SB_SYNC_TAIL;

	gSerialBridgeBusyTx = true;
	BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);

	BK4819_WriteRegister(BK4819_REG_70, 0x00E0);
	RADIO_SetTxParameters();
	BK4819_SendFSKData(gSerialBridgeFSKBuffer);

	BK4819_SetupPowerAmplifier(0, 0);
	BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, false);
	BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);

	gSerialBridgeTxPackets++;
	gSerialBridgeTxBytes += len;
	s_tx_seq++;
	s_tx_gap = SB_TX_GAP;
	s_pending_rearm = true;

	if (len < s_uart_len) {
		memmove(s_uart_rx, s_uart_rx + len, s_uart_len - len);
		s_uart_len -= len;
	} else {
		s_uart_len = 0;
	}

	SB_DiscardUartNoise();
	RADIO_SetupRegisters(true);
	gUpdateDisplay = true;
}

bool SERIAL_BRIDGE_IsActive(void)
{
	return s_active;
}

bool SERIAL_BRIDGE_IsAnalogRx(void)
{
	return s_active && SB_AnalogRxActive();
}

bool SERIAL_BRIDGE_BlockAnalogPtt(void)
{
#ifndef ENABLE_ANALOG_PTT
	if (!s_active)
		return true;
#endif
	return false;
}

void SERIAL_BRIDGE_Start(void)
{
	s_uart_len     = 0;
	s_idle_ticks   = 0;
	s_tx_gap       = 0;
	s_busy_wait    = 0;
	s_pending_rearm = false;
	s_have_rx_seq  = false;
	s_uart_dma_idx = 0;
	gSerialBridgeTxPackets = 0;
	gSerialBridgeRxPackets = 0;
	gSerialBridgeRxErrors  = 0;
	gSerialBridgeTxBytes   = 0;
	gSerialBridgeRxBytes   = 0;
	gSerialBridgeBusyTx    = false;
	gInputBoxIndex = 0;
	s_active = true;

	s_saved_dual_watch = gEeprom.DUAL_WATCH;
	s_saved_cross_band = gEeprom.CROSS_BAND_RX_TX;
	gEeprom.DUAL_WATCH = DUAL_WATCH_OFF;
	gEeprom.CROSS_BAND_RX_TX = CROSS_BAND_OFF;

	RADIO_SelectVfos();
	GUI_SelectNextDisplay(DISPLAY_SERIAL_BRIDGE);
	gRequestDisplayScreen = DISPLAY_SERIAL_BRIDGE;
	GUI_DisplayScreen();
	UI_DisplayStatus();

	SB_ApplyFrequency(SERIAL_BRIDGE_DEFAULT_FREQ);
	s_uart_dma_idx = SB_UartDmaWriteIndex();
	gUpdateDisplay = true;
	gUpdateStatus  = true;
}

void SERIAL_BRIDGE_Stop(void)
{
	s_active = false;
	gSerialBridgeBusyTx = false;
	s_pending_rearm = false;
	gInputBoxIndex = 0;
	gEeprom.DUAL_WATCH = s_saved_dual_watch;
	gEeprom.CROSS_BAND_RX_TX = s_saved_cross_band;
	BK4819_ResetFSK();
	RADIO_SelectVfos();
	RADIO_SetupRegisters(true);
	gRequestDisplayScreen = DISPLAY_MAIN;
	gUpdateDisplay = true;
	gUpdateStatus  = true;
}

void SERIAL_BRIDGE_StorePacket(void)
{
	uint8_t  seq;
	uint8_t  len;
	uint16_t status;
	uint16_t crc;
	const uint8_t *payload;

	if (gFSKWriteIndex < 36)
		return;

	gFSKWriteIndex = 0;
	status = BK4819_ReadRegister(BK4819_REG_0B);

	if (SB_AnalogBusy() || SB_AnalogRxActive() || g_SquelchLost)
		return;

	SERIAL_BRIDGE_ReArm();

	if (gSerialBridgeBusyTx || s_tx_gap > 0)
		return;

	if ((status & 0x0010U) != 0 || gSerialBridgeFSKBuffer[0] != SB_SYNC_HEAD || gSerialBridgeFSKBuffer[35] != SB_SYNC_TAIL) {
		gSerialBridgeRxErrors++;
		gUpdateDisplay = true;
		return;
	}

	crc = CRC_Calculate(&gSerialBridgeFSKBuffer[1], 2 + 64);
	if (gSerialBridgeFSKBuffer[34] != crc) {
		gSerialBridgeRxErrors++;
		gUpdateDisplay = true;
		return;
	}

	seq = (uint8_t)(gSerialBridgeFSKBuffer[1] >> 8);
	len = (uint8_t)(gSerialBridgeFSKBuffer[1] & 0xFF);
	if (len == 0 || len > SERIAL_BRIDGE_PAYLOAD_MAX) {
		gSerialBridgeRxErrors++;
		gUpdateDisplay = true;
		return;
	}

	if (seq == (uint8_t)(s_tx_seq - 1u))
		return;

	if (s_have_rx_seq && seq == s_last_rx_seq) {
		gUpdateDisplay = true;
		return;
	}
	s_last_rx_seq = seq;
	s_have_rx_seq = true;

	payload = (const uint8_t *)&gSerialBridgeFSKBuffer[2];
	UART_Send(payload, len);

	gSerialBridgeRxPackets++;
	gSerialBridgeRxBytes += len;
	gUpdateDisplay = true;
}

void SERIAL_BRIDGE_TimeSlice10ms(void)
{
	uint8_t chunk;

	if (!s_active)
		return;

	if (s_tx_gap > 0) {
		s_tx_gap--;
		SB_DiscardUartNoise();
		gFSKWriteIndex = 0;
		if (s_tx_gap == 0)
			gSerialBridgeBusyTx = false;
		else
			return;
	}

	if (SB_AnalogBusy() || SB_AnalogRxActive()) {
		SB_DrainUart();
		return;
	}

	if (!g_SquelchLost && gEnableSpeaker)
		SB_SpeakerOff();

	if (s_pending_rearm
	    && !g_SquelchLost
	    && !gEndOfRxDetectedMaybe
	    && gTailNoteEliminationCountdown_10ms == 0)
		SERIAL_BRIDGE_ReArm();

	if (s_tx_gap > 0)
		return;

	SB_DrainUart();

	if (s_uart_len == 0) {
		s_idle_ticks = 0;
		return;
	}

	s_idle_ticks++;

	chunk = s_uart_len;
	if (chunk > SERIAL_BRIDGE_PAYLOAD_MAX)
		chunk = SERIAL_BRIDGE_PAYLOAD_MAX;

	if (s_uart_len < SERIAL_BRIDGE_PAYLOAD_MAX && s_idle_ticks < SB_IDLE_TICKS)
		return;

	if (g_SquelchLost && s_busy_wait < SB_BUSY_WAIT) {
		s_busy_wait++;
		return;
	}
	s_busy_wait = 0;

	SB_SendPayload(s_uart_rx, chunk);
}

static void SB_KeyDigits(KEY_Code_t Key)
{
	INPUTBOX_Append(Key);
	gUpdateDisplay = true;

	if (gInputBoxIndex < 6)
		return;

	uint32_t Frequency = StrToUL(INPUTBOX_GetAscii()) * 100;
	gInputBoxIndex = 0;

	if (RX_freq_check(Frequency) != 0) {
		gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
		gUpdateDisplay = true;
		return;
	}

	SB_ApplyFrequency(Frequency);
}

static void SB_KeyUpDown(int8_t Direction)
{
	uint32_t Frequency;

	if (gInputBoxIndex > 0) {
		gInputBoxIndex = 0;
		gUpdateDisplay = true;
		return;
	}

	Frequency = APP_SetFrequencyByStep(gTxVfo, Direction);
	SB_ApplyFrequency(Frequency);
}

void SERIAL_BRIDGE_ProcessKeys(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld)
{
	if (Key == KEY_PTT) {
		GENERIC_Key_PTT(bKeyPressed);
		return;
	}

	if (bKeyHeld || !bKeyPressed)
		return;

	/* PTT can ghost the keypad matrix; ignore other keys while keyed. */
	if (gPttIsPressed)
		return;

	if (Key <= KEY_9) {
		SB_KeyDigits(Key);
		return;
	}

	switch (Key) {
	case KEY_EXIT:
		if (gInputBoxIndex > 0) {
			gInputBox[--gInputBoxIndex] = 10;
			gUpdateDisplay = true;
		} else {
			SERIAL_BRIDGE_Stop();
		}
		break;

	case KEY_UP:
		SB_KeyUpDown(+1);
		break;

	case KEY_DOWN:
		SB_KeyUpDown(-1);
		break;

	case KEY_MENU:
		s_idle_ticks = SB_IDLE_TICKS;
		break;

	default:
		break;
	}
}

#endif /* ENABLE_SERIAL_BRIDGE */
