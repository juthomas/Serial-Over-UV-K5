/* Serial Bridge — UART <-> FSK / DTMF RF tunnel for UV-K5 V3 (PY32F071)
 *
 * Same air frames as V1. UART RX uses PY32 DMA remaining-count.
 *
 * FSK frame (72 bytes / 36 x uint16_t), same length as Air Copy:
 *   [0]      = 0xABCD
 *   [1]      = (seq << 8) | len   (len = 1..56)
 *   [2..29]  = payload (up to 56 bytes)
 *   [30..33] = padding (0)
 *   [34]     = CRC16-CCITT over bytes of words [1..33] (66 bytes)
 *   [35]     = 0xDCBA
 *
 * DTMF frame (nibbles, ~80+80 ms per symbol):
 *   * *  LEN(1..8)  SEQ  DATA(2*LEN)  CRC8(2)
 *
 * Runtime * toggles FSK / DTMF. XOR: data UART at idle (speaker off);
 * analog FM during a QSO (data modem paused).
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
#define SB_UART_BUF    256u
#define SB_IDLE_TICKS  3u   /* ~30 ms at 10 ms slice before FSK flush */
#define SB_TX_GAP      20u  /* ~200 ms mute after FSK TX */
#define SB_BUSY_WAIT   20u  /* max ~200 ms wait if channel busy */
#define SB_XON         0x11u
#define SB_XOFF        0x13u

#define SB_MODE_FSK    0u
#define SB_MODE_DTMF   1u

#define SB_DTMF_TONE_TICKS   8u   /* 80 ms */
#define SB_DTMF_GAP_TICKS    8u   /* 80 ms */
#define SB_DTMF_KEYUP_TICKS  5u   /* 50 ms after PA up */
#define SB_DTMF_IDLE_TICKS   8u   /* 80 ms UART idle flush */
#define SB_DTMF_TX_GAP       40u  /* 400 ms ignore own echo */
#define SB_DTMF_RX_TIMEOUT   40u  /* 400 ms without a symbol */
#define SB_DTMF_RECENT       30u  /* 300 ms: treat RX as data, not voice */
#define SB_DTMF_SYMS_MAX     22u  /* ** + LEN + SEQ + 16 data + 2 CRC */
#define SB_DTMF_XOFF_WATER   32u
#define SB_UART_DMA_CH LL_DMA_CHANNEL_2

enum {
	SB_DTMF_TX_IDLE = 0,
	SB_DTMF_TX_KEYUP,
	SB_DTMF_TX_SYM,
	SB_DTMF_TX_PAUSE
};

enum {
	SB_DTMF_RX_SYNC0 = 0,
	SB_DTMF_RX_SYNC1,
	SB_DTMF_RX_LEN,
	SB_DTMF_RX_SEQ,
	SB_DTMF_RX_DATA,
	SB_DTMF_RX_CRC0,
	SB_DTMF_RX_CRC1
};

static uint8_t  s_uart_rx[SB_UART_BUF];
static uint16_t s_uart_len;
static uint16_t s_uart_dma_idx;
static uint8_t  s_idle_ticks;
static uint8_t  s_tx_gap;
static uint8_t  s_busy_wait;
static uint8_t  s_tx_seq;
static uint8_t  s_last_rx_seq;
static bool     s_have_rx_seq;
static bool     s_active;
static bool     s_pending_rearm;
static bool     s_uart_xoff;
static uint8_t  s_saved_dual_watch;
static uint8_t  s_saved_cross_band;
static uint8_t  s_mode = SERIAL_BRIDGE_DEFAULT_MODE;

static uint8_t  s_dtmf_tx_state;
static uint8_t  s_dtmf_tx_ticks;
static uint8_t  s_dtmf_tx_idx;
static uint8_t  s_dtmf_tx_nsyms;
static uint8_t  s_dtmf_tx_bytes;
static char     s_dtmf_tx_syms[SB_DTMF_SYMS_MAX];

static uint8_t  s_dtmf_rx_state;
static uint8_t  s_dtmf_rx_age;
static uint8_t  s_dtmf_rx_recent;
static uint8_t  s_dtmf_rx_len;
static uint8_t  s_dtmf_rx_seq;
static uint8_t  s_dtmf_rx_need;
static uint8_t  s_dtmf_rx_got;
static uint8_t  s_dtmf_rx_crc_hi;
static uint8_t  s_dtmf_rx_data[SERIAL_BRIDGE_DTMF_PAYLOAD_MAX];

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

static uint8_t SB_PayloadMax(void)
{
	return (s_mode == SB_MODE_DTMF) ? SERIAL_BRIDGE_DTMF_PAYLOAD_MAX : SERIAL_BRIDGE_PAYLOAD_MAX;
}

static uint8_t SB_IdleFlushTicks(void)
{
	return (s_mode == SB_MODE_DTMF) ? SB_DTMF_IDLE_TICKS : SB_IDLE_TICKS;
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
	/* Idle overlay: analog demod stays configured, speaker stays off. */
	BK4819_WriteRegister(BK4819_REG_70, 0x0000);
	RADIO_SetModulation(gRxVfo->Modulation);
	BK4819_WriteRegister(BK4819_REG_48,
		(11u << 12) |
		( 0u << 10) |
		(gEeprom.VOLUME_GAIN << 4) |
		(gEeprom.DAC_GAIN << 0));
	SB_SpeakerOff();
}

static void SB_UartSendByte(uint8_t b)
{
	UART_Send(&b, 1);
}

static void SB_UartXoff(void)
{
	if (s_uart_xoff)
		return;
	s_uart_xoff = true;
	SB_UartSendByte(SB_XOFF);
}

static void SB_UartXon(void)
{
	if (!s_uart_xoff)
		return;
	s_uart_xoff = false;
	SB_UartSendByte(SB_XON);
}

static void SB_DrainUart(void)
{
	const uint16_t dma_len = SB_UartDmaWriteIndex();
	const bool drop_noise = gSerialBridgeBusyTx || s_tx_gap > 0;

	while (s_uart_dma_idx != dma_len) {
		const uint8_t b = UART_DMA_Buffer[s_uart_dma_idx];

		/* EMI during TX is 0xFF (CH340) or 0x00 (PL2303). */
		if (drop_noise && (b == 0x00u || b == 0xFFu)) {
			s_uart_dma_idx = (uint16_t)((s_uart_dma_idx + 1u) % sizeof(UART_DMA_Buffer));
			continue;
		}
		if (s_uart_len >= SB_UART_BUF) {
			SB_UartXoff();
			break;
		}
		s_uart_rx[s_uart_len++] = b;
		s_uart_dma_idx = (uint16_t)((s_uart_dma_idx + 1u) % sizeof(UART_DMA_Buffer));
		s_idle_ticks = 0;
		if (s_mode == SB_MODE_DTMF && s_uart_len >= SB_DTMF_XOFF_WATER)
			SB_UartXoff();
	}
}

static bool SB_AnalogBusy(void)
{
	return gPttIsPressed || gCurrentFunction == FUNCTION_TRANSMIT;
}

static void SB_ConsumeUart(uint8_t len)
{
	if (len < s_uart_len) {
		memmove(s_uart_rx, s_uart_rx + len, s_uart_len - len);
		s_uart_len -= len;
	} else {
		s_uart_len = 0;
	}
}

static void SB_ApplyFrequency(uint32_t Frequency)
{
	const uint8_t Vfo = gEeprom.TX_VFO;
	const FREQUENCY_Band_t band = FREQUENCY_GetBand(Frequency);
	uint16_t step = gTxVfo->StepFrequency;

	/* 8.33 kHz uses an aviation channel scheme — not for a typed VFO freq. */
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

static char SB_NibbleToSym(uint8_t n)
{
	n &= 0x0Fu;
	if (n < 10)
		return (char)('0' + n);
	if (n < 14)
		return (char)('A' + (n - 10));
	return (n == 14) ? '*' : '#';
}

static int8_t SB_SymToNibble(char c)
{
	if (c >= '0' && c <= '9')
		return (int8_t)(c - '0');
	if (c >= 'A' && c <= 'D')
		return (int8_t)(c - 'A' + 10);
	if (c == '*')
		return 14;
	if (c == '#')
		return 15;
	return -1;
}

static uint8_t SB_Crc8(const uint8_t *data, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
		uint8_t i;

		crc ^= *data++;
		for (i = 0; i < 8; i++)
			crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
	}
	return crc;
}

static void SB_DtmfRxReset(void)
{
	s_dtmf_rx_state = SB_DTMF_RX_SYNC0;
	s_dtmf_rx_age   = 0;
	s_dtmf_rx_len   = 0;
	s_dtmf_rx_need  = 0;
	s_dtmf_rx_got   = 0;
}

static void SB_DtmfRxTick(void)
{
	if (s_dtmf_rx_recent > 0)
		s_dtmf_rx_recent--;

	if (s_dtmf_rx_state == SB_DTMF_RX_SYNC0)
		return;

	if (++s_dtmf_rx_age >= SB_DTMF_RX_TIMEOUT)
		SB_DtmfRxReset();
}

static void SB_DtmfTxAbort(void)
{
	if (s_dtmf_tx_state == SB_DTMF_TX_IDLE)
		return;

	BK4819_ExitDTMF_TX(true);
	BK4819_WriteRegister(BK4819_REG_70, 0x0000);
	s_dtmf_tx_state = SB_DTMF_TX_IDLE;
	s_dtmf_tx_nsyms = 0;
	gSerialBridgeBusyTx = false;
}

static void SB_DtmfTxFinish(void)
{
	BK4819_ExitDTMF_TX(true);
	BK4819_SetupPowerAmplifier(0, 0);
	BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, false);
	BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);

	gSerialBridgeTxPackets++;
	gSerialBridgeTxBytes += s_dtmf_tx_bytes;
	s_tx_seq++;
	SB_ConsumeUart(s_dtmf_tx_bytes);

	s_dtmf_tx_state = SB_DTMF_TX_IDLE;
	s_dtmf_tx_nsyms = 0;
	s_tx_gap = SB_DTMF_TX_GAP;
	s_pending_rearm = true;
	/* Keep BusyTx until the mute window ends (echo + USB EMI). */

	SB_DrainUart();
	RADIO_SetupRegisters(true);
	gUpdateDisplay = true;
}

static uint8_t SB_DtmfEncode(const uint8_t *data, uint8_t len)
{
	uint8_t crc;
	uint8_t n = 0;
	uint8_t i;
	uint8_t seq = (uint8_t)(s_tx_seq & 0x0Fu);
	uint8_t tmp[2 + SERIAL_BRIDGE_DTMF_PAYLOAD_MAX];

	if (len == 0 || len > SERIAL_BRIDGE_DTMF_PAYLOAD_MAX)
		return 0;

	tmp[0] = len;
	tmp[1] = seq;
	memcpy(tmp + 2, data, len);
	crc = SB_Crc8(tmp, (uint8_t)(2u + len));

	s_dtmf_tx_syms[n++] = '*';
	s_dtmf_tx_syms[n++] = '*';
	s_dtmf_tx_syms[n++] = SB_NibbleToSym(len);
	s_dtmf_tx_syms[n++] = SB_NibbleToSym(seq);
	for (i = 0; i < len; i++) {
		s_dtmf_tx_syms[n++] = SB_NibbleToSym((uint8_t)(data[i] >> 4));
		s_dtmf_tx_syms[n++] = SB_NibbleToSym((uint8_t)(data[i] & 0x0Fu));
	}
	s_dtmf_tx_syms[n++] = SB_NibbleToSym((uint8_t)(crc >> 4));
	s_dtmf_tx_syms[n++] = SB_NibbleToSym((uint8_t)(crc & 0x0Fu));
	return n;
}

static void SB_DtmfTxStart(const uint8_t *data, uint8_t len)
{
	s_dtmf_tx_nsyms = SB_DtmfEncode(data, len);
	if (s_dtmf_tx_nsyms == 0)
		return;

	s_dtmf_tx_bytes = len;
	s_dtmf_tx_idx   = 0;
	s_dtmf_tx_ticks = SB_DTMF_KEYUP_TICKS;
	s_dtmf_tx_state = SB_DTMF_TX_KEYUP;

	gSerialBridgeBusyTx = true;
	SB_UartXoff();
	BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);
	RADIO_SetTxParameters();
	BK4819_EnterDTMF_TX(false);
}

static void SB_DtmfTxTick(void)
{
	if (s_dtmf_tx_ticks > 0) {
		s_dtmf_tx_ticks--;
		return;
	}

	switch (s_dtmf_tx_state) {
	case SB_DTMF_TX_KEYUP:
		BK4819_PlayDTMF(s_dtmf_tx_syms[s_dtmf_tx_idx]);
		BK4819_ExitTxMute();
		s_dtmf_tx_state = SB_DTMF_TX_SYM;
		s_dtmf_tx_ticks = SB_DTMF_TONE_TICKS;
		break;

	case SB_DTMF_TX_SYM:
		BK4819_EnterTxMute();
		s_dtmf_tx_idx++;
		s_dtmf_tx_state = SB_DTMF_TX_PAUSE;
		s_dtmf_tx_ticks = SB_DTMF_GAP_TICKS;
		break;

	case SB_DTMF_TX_PAUSE:
		if (s_dtmf_tx_idx >= s_dtmf_tx_nsyms) {
			SB_DtmfTxFinish();
			break;
		}
		BK4819_PlayDTMF(s_dtmf_tx_syms[s_dtmf_tx_idx]);
		BK4819_ExitTxMute();
		s_dtmf_tx_state = SB_DTMF_TX_SYM;
		s_dtmf_tx_ticks = SB_DTMF_TONE_TICKS;
		break;

	default:
		break;
	}
}

static void SB_DtmfDeliver(void)
{
	uint8_t tmp[2 + SERIAL_BRIDGE_DTMF_PAYLOAD_MAX];
	uint8_t crc;
	uint8_t expect;

	tmp[0] = s_dtmf_rx_len;
	tmp[1] = s_dtmf_rx_seq;
	memcpy(tmp + 2, s_dtmf_rx_data, s_dtmf_rx_len);
	crc = SB_Crc8(tmp, (uint8_t)(2u + s_dtmf_rx_len));
	expect = (uint8_t)((s_dtmf_rx_crc_hi << 4) | (uint8_t)(s_dtmf_rx_got & 0x0Fu));

	if (crc != expect) {
		gSerialBridgeRxErrors++;
		gUpdateDisplay = true;
		SB_DtmfRxReset();
		return;
	}

	/* Own packet leaking back */
	if (s_dtmf_rx_seq == (uint8_t)((s_tx_seq - 1u) & 0x0Fu)) {
		SB_DtmfRxReset();
		return;
	}

	if (s_have_rx_seq && s_dtmf_rx_seq == (s_last_rx_seq & 0x0Fu)) {
		SB_DtmfRxReset();
		gUpdateDisplay = true;
		return;
	}
	s_last_rx_seq = s_dtmf_rx_seq;
	s_have_rx_seq = true;

	UART_Send(s_dtmf_rx_data, s_dtmf_rx_len);
	gSerialBridgeRxPackets++;
	gSerialBridgeRxBytes += s_dtmf_rx_len;
	gUpdateDisplay = true;
	SB_DtmfRxReset();
}

void SERIAL_BRIDGE_OnDtmf(char c)
{
	int8_t n;

	if (!s_active || s_mode != SB_MODE_DTMF)
		return;
	if (gSerialBridgeBusyTx || s_tx_gap > 0 || SB_AnalogBusy())
		return;
	if (SB_AnalogRxActive() && s_dtmf_rx_state == SB_DTMF_RX_SYNC0 && s_dtmf_rx_recent == 0)
		return;

	n = SB_SymToNibble(c);
	if (n < 0)
		return;

	s_dtmf_rx_recent = SB_DTMF_RECENT;
	s_dtmf_rx_age    = 0;
	SB_SpeakerOff();

	switch (s_dtmf_rx_state) {
	case SB_DTMF_RX_SYNC0:
		if (c == '*')
			s_dtmf_rx_state = SB_DTMF_RX_SYNC1;
		break;

	case SB_DTMF_RX_SYNC1:
		s_dtmf_rx_state = (c == '*') ? SB_DTMF_RX_LEN : SB_DTMF_RX_SYNC0;
		break;

	case SB_DTMF_RX_LEN:
		if (n < 1 || n > (int8_t)SERIAL_BRIDGE_DTMF_PAYLOAD_MAX) {
			gSerialBridgeRxErrors++;
			gUpdateDisplay = true;
			SB_DtmfRxReset();
			break;
		}
		s_dtmf_rx_len  = (uint8_t)n;
		s_dtmf_rx_need = (uint8_t)(n * 2u);
		s_dtmf_rx_got  = 0;
		s_dtmf_rx_state = SB_DTMF_RX_SEQ;
		break;

	case SB_DTMF_RX_SEQ:
		s_dtmf_rx_seq   = (uint8_t)n;
		s_dtmf_rx_state = SB_DTMF_RX_DATA;
		break;

	case SB_DTMF_RX_DATA:
		if ((s_dtmf_rx_got & 1u) == 0)
			s_dtmf_rx_data[s_dtmf_rx_got / 2u] = (uint8_t)((uint8_t)n << 4);
		else
			s_dtmf_rx_data[s_dtmf_rx_got / 2u] |= (uint8_t)n;
		s_dtmf_rx_got++;
		if (s_dtmf_rx_got >= s_dtmf_rx_need)
			s_dtmf_rx_state = SB_DTMF_RX_CRC0;
		break;

	case SB_DTMF_RX_CRC0:
		s_dtmf_rx_crc_hi = (uint8_t)n;
		s_dtmf_rx_state  = SB_DTMF_RX_CRC1;
		break;

	case SB_DTMF_RX_CRC1:
		s_dtmf_rx_got = (uint8_t)n; /* reuse as CRC low */
		SB_DtmfDeliver();
		break;

	default:
		SB_DtmfRxReset();
		break;
	}
}

static void SB_ToggleMode(void)
{
	if (s_dtmf_tx_state != SB_DTMF_TX_IDLE)
		SB_DtmfTxAbort();

	SB_DtmfRxReset();
	s_dtmf_rx_recent = 0;
	s_have_rx_seq = false;
	gFSKWriteIndex = 0;
	s_mode = (s_mode == SB_MODE_DTMF) ? SB_MODE_FSK : SB_MODE_DTMF;

	BK4819_WriteRegister(BK4819_REG_58, 0x0000);
	BK4819_WriteRegister(BK4819_REG_70, 0x0000);
	BK4819_WriteRegister(BK4819_REG_59, 0x0068);
	s_pending_rearm = true;
	SERIAL_BRIDGE_ReArm();

	gBeepToPlay = BEEP_1KHZ_60MS_OPTIONAL;
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

	if (s_mode == SB_MODE_DTMF) {
		BK4819_WriteRegister(BK4819_REG_58, 0x0000);
		BK4819_WriteRegister(BK4819_REG_70, 0x0000);
		BK4819_EnableDTMF();
		mask = BK4819_ReadRegister(BK4819_REG_3F);
		mask |= (uint16_t)(
			BK4819_REG_3F_DTMF_5TONE_FOUND |
			BK4819_REG_3F_SQUELCH_FOUND |
			BK4819_REG_3F_SQUELCH_LOST |
			BK4819_REG_3F_CxCSS_TAIL);
		BK4819_WriteRegister(BK4819_REG_3F, mask);
		SB_RestoreAnalogRx();
		return;
	}

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

	if (s_dtmf_tx_state != SB_DTMF_TX_IDLE)
		SB_DtmfTxAbort();

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
	SB_DtmfRxReset();

	/* Disable FSK modem only. Do not write REG_59=0x0068: that mutes AF on V1. */
	BK4819_WriteRegister(BK4819_REG_58, 0x0000);
	BK4819_WriteRegister(BK4819_REG_70, 0x0000);
	BK4819_RX_TurnOn();
}

void SERIAL_BRIDGE_HoldAfterTx(void)
{
	if (!s_active)
		return;

	s_tx_gap = (s_mode == SB_MODE_DTMF) ? SB_DTMF_TX_GAP : SB_TX_GAP;
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
	/* Keep BusyTx set until the mute window ends so FSK RX + UART EMI
	 * cannot bounce back to the PC as \xff. */

	SB_ConsumeUart(len);
	SB_DrainUart();
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

bool SERIAL_BRIDGE_IsDtmf(void)
{
	return s_active && s_mode == SB_MODE_DTMF;
}

bool SERIAL_BRIDGE_HoldDataRx(void)
{
	return s_active
	    && s_mode == SB_MODE_DTMF
	    && (s_dtmf_rx_recent > 0 || s_dtmf_rx_state != SB_DTMF_RX_SYNC0);
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
	s_uart_xoff    = false;
	s_uart_dma_idx = 0;
	s_dtmf_tx_state = SB_DTMF_TX_IDLE;
	SB_DtmfRxReset();
	s_dtmf_rx_recent = 0;
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
	if (s_dtmf_tx_state != SB_DTMF_TX_IDLE)
		SB_DtmfTxAbort();

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

	if (s_mode == SB_MODE_DTMF)
		return;

	if (gFSKWriteIndex < 36)
		return;

	gFSKWriteIndex = 0;
	status = BK4819_ReadRegister(BK4819_REG_0B);

	if (SB_AnalogBusy() || SB_AnalogRxActive() || g_SquelchLost)
		return;

	SERIAL_BRIDGE_ReArm();

	/* Ignore leftover FIFO / own echo while PA is still ringing. */
	if (gSerialBridgeBusyTx || s_tx_gap > 0)
		return;

	/* Same REG_0B quirk as Air Copy: bit4 set means CRC fail in practice */
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

	/* Own packet leaking back through the FSK RX path */
	if (seq == (uint8_t)(s_tx_seq - 1u))
		return;

	/* Drop duplicate retransmits */
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
	uint8_t payload_max;
	uint8_t idle_need;

	if (!s_active)
		return;

	SB_DtmfRxTick();

	if (s_mode == SB_MODE_DTMF && s_dtmf_tx_state != SB_DTMF_TX_IDLE) {
		if (SB_AnalogBusy()) {
			SB_DtmfTxAbort();
			SB_DrainUart();
			return;
		}
		SB_DtmfTxTick();
		SB_DrainUart();
		return;
	}

	if (s_tx_gap > 0) {
		s_tx_gap--;
		SB_DrainUart();
		gFSKWriteIndex = 0;
		if (s_tx_gap == 0) {
			gSerialBridgeBusyTx = false;
			if (s_uart_len < SB_PayloadMax())
				SB_UartXon();
		} else
			return;
	}

	if (SB_AnalogBusy()) {
		SB_DrainUart();
		return;
	}

	if (SB_AnalogRxActive()) {
		if (SERIAL_BRIDGE_HoldDataRx())
			SB_SpeakerOff();
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

	payload_max = SB_PayloadMax();
	idle_need   = SB_IdleFlushTicks();
	chunk = (s_uart_len > payload_max) ? payload_max : (uint8_t)s_uart_len;

	/* Flush when buffer full or idle timeout after first byte */
	if (s_uart_len < payload_max && s_idle_ticks < idle_need)
		return;

	/* Simple CSMA: defer TX while squelch open (someone transmitting) */
	if (g_SquelchLost && s_busy_wait < SB_BUSY_WAIT) {
		s_busy_wait++;
		return;
	}
	s_busy_wait = 0;

	SB_UartXoff();
	if (s_mode == SB_MODE_DTMF)
		SB_DtmfTxStart(s_uart_rx, chunk);
	else
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

	case KEY_STAR:
		if (gInputBoxIndex == 0)
			SB_ToggleMode();
		break;

	case KEY_MENU:
		/* Force flush any pending UART data */
		s_idle_ticks = SB_IdleFlushTicks();
		break;

	default:
		break;
	}
}

#endif /* ENABLE_SERIAL_BRIDGE */
