/* Serial Bridge — UART <-> FSK RF tunnel for UV-K5 (egzumer-based)
 *
 * Frame (72 bytes / 36 x uint16_t), same FSK length as Air Copy:
 *   [0]      = 0xABCD
 *   [1]      = (seq << 8) | len   (len = 1..56)
 *   [2..29]  = payload (up to 56 bytes)
 *   [30..33] = padding (0)
 *   [34]     = CRC16-CCITT over bytes of words [1..33] (66 bytes)
 *   [35]     = 0xDCBA
 */

#ifdef ENABLE_SERIAL_BRIDGE

#include "app/serial_bridge.h"

#include <string.h>

#include "bsp/dp32g030/dma.h"
#include "driver/bk4819.h"
#include "driver/crc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "misc.h"
#include "radio.h"
#include "ui/ui.h"

#define SB_SYNC_HEAD   0xABCDu
#define SB_SYNC_TAIL   0xDCBAu
#define SB_UART_BUF    128u
#define SB_IDLE_TICKS  3u   /* ~30 ms at 10 ms slice before flush */
#define SB_TX_GAP      20u  /* ~200 ms mute after TX: drop self-RX + USB EMI */
#define SB_BUSY_WAIT   20u  /* max ~200 ms wait if channel busy */

static uint8_t  s_uart_rx[SB_UART_BUF];
static uint8_t  s_uart_len;
static uint8_t  s_uart_dma_idx;
static uint8_t  s_idle_ticks;
static uint8_t  s_tx_gap;
static uint8_t  s_busy_wait;
static uint8_t  s_tx_seq;
static uint8_t  s_last_rx_seq;
static bool     s_have_rx_seq;
static bool     s_active;

uint16_t gSerialBridgeFSKBuffer[36];
uint16_t gSerialBridgeTxPackets;
uint16_t gSerialBridgeRxPackets;
uint16_t gSerialBridgeRxErrors;
uint16_t gSerialBridgeTxBytes;
uint16_t gSerialBridgeRxBytes;
bool     gSerialBridgeBusyTx;

static void SB_SetupFSK(void)
{
	/* Same register recipe as Air Copy / BK4819_SetupAircopy */
	BK4819_WriteRegister(BK4819_REG_70, 0x00E0);
	BK4819_WriteRegister(BK4819_REG_72, 0x3065);
	BK4819_WriteRegister(BK4819_REG_58, 0x00C1);
	BK4819_WriteRegister(BK4819_REG_5C, 0x5665);
	BK4819_WriteRegister(BK4819_REG_5D, 0x4700);
}

static void SB_ArmRx(void)
{
	gFSKWriteIndex = 0;
	BK4819_PrepareFSKReceive();
}

static void SB_DiscardUartNoise(void)
{
	/* EMI during FSK TX shows up as 0xFF on the programming cable UART.
	 * Throw those bytes away so we neither echo them to the PC nor
	 * re-transmit them as a new FSK frame. */
	s_uart_dma_idx = (uint8_t)(DMA_CH0->ST & 0xFFFU);
}

static void SB_DrainUart(void)
{
	const uint16_t dma_len = DMA_CH0->ST & 0xFFFU;

	while (s_uart_dma_idx != dma_len && s_uart_len < SB_UART_BUF) {
		s_uart_rx[s_uart_len++] = UART_DMA_Buffer[s_uart_dma_idx];
		s_uart_dma_idx = (uint8_t)((s_uart_dma_idx + 1u) % sizeof(UART_DMA_Buffer));
		s_idle_ticks = 0;
	}
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

	RADIO_SetTxParameters();
	BK4819_SendFSKData(gSerialBridgeFSKBuffer);

	BK4819_SetupPowerAmplifier(0, 0);
	BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, false);
	BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);

	gSerialBridgeTxPackets++;
	gSerialBridgeTxBytes += len;
	s_tx_seq++;
	s_tx_gap = SB_TX_GAP;
	/* Keep BusyTx set until the mute window ends so FSK RX + UART EMI
	 * cannot bounce back to the PC as \xff. */

	/* shift remaining UART bytes */
	if (len < s_uart_len) {
		memmove(s_uart_rx, s_uart_rx + len, s_uart_len - len);
		s_uart_len -= len;
	} else {
		s_uart_len = 0;
	}

	SB_DiscardUartNoise();
	SB_SetupFSK();
	SB_ArmRx();
	gUpdateDisplay = true;
}

bool SERIAL_BRIDGE_IsActive(void)
{
	return s_active;
}

bool SERIAL_BRIDGE_BlockAnalogPtt(void)
{
	/* Analog FM TX next to a USB programming cable sprays RF into the host
	 * (Mac screen glitches, USB disconnects). Block it unless the build
	 * explicitly re-enables analog PTT, and always block in bridge mode. */
#ifndef ENABLE_ANALOG_PTT
	return true;
#else
	return s_active;
#endif
}

void SERIAL_BRIDGE_Start(void)
{
	s_uart_len     = 0;
	s_idle_ticks   = 0;
	s_tx_gap       = 0;
	s_busy_wait    = 0;
	s_have_rx_seq  = false;
	s_uart_dma_idx = (uint8_t)(DMA_CH0->ST & 0xFFFU);
	gSerialBridgeTxPackets = 0;
	gSerialBridgeRxPackets = 0;
	gSerialBridgeRxErrors  = 0;
	gSerialBridgeTxBytes   = 0;
	gSerialBridgeRxBytes   = 0;
	gSerialBridgeBusyTx    = false;
	s_active = true;

	RADIO_SelectVfos();
	RADIO_SetupRegisters(true);
	SB_SetupFSK();
	SB_ArmRx();

	GUI_SelectNextDisplay(DISPLAY_SERIAL_BRIDGE);
	gUpdateStatus  = true;
}

void SERIAL_BRIDGE_Stop(void)
{
	s_active = false;
	gSerialBridgeBusyTx = false;
	BK4819_ResetFSK();
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
	SB_ArmRx();

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

	if (!s_active)
		return;

	if (s_tx_gap > 0) {
		s_tx_gap--;
		SB_DiscardUartNoise();
		gFSKWriteIndex = 0;
		if (s_tx_gap == 0)
			gSerialBridgeBusyTx = false;
		return;
	}

	SB_DrainUart();

	if (s_uart_len == 0) {
		s_idle_ticks = 0;
		return;
	}

	s_idle_ticks++;

	chunk = s_uart_len;
	if (chunk > SERIAL_BRIDGE_PAYLOAD_MAX)
		chunk = SERIAL_BRIDGE_PAYLOAD_MAX;

	/* Flush when buffer full or idle timeout after first byte */
	if (s_uart_len < SERIAL_BRIDGE_PAYLOAD_MAX && s_idle_ticks < SB_IDLE_TICKS)
		return;

	/* Simple CSMA: defer TX while squelch open (someone transmitting) */
	if (g_SquelchLost && s_busy_wait < SB_BUSY_WAIT) {
		s_busy_wait++;
		return;
	}
	s_busy_wait = 0;

	SB_SendPayload(s_uart_rx, chunk);
}

void SERIAL_BRIDGE_ProcessKeys(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld)
{
	if (bKeyHeld || !bKeyPressed)
		return;

	switch (Key) {
	case KEY_EXIT:
		SERIAL_BRIDGE_Stop();
		break;

	case KEY_MENU:
		/* Force flush any pending UART data */
		s_idle_ticks = SB_IDLE_TICKS;
		break;

	case KEY_PTT:
		/* Physical PTT is ignored: analog TX would RFI the USB host. */
		break;

	default:
		break;
	}
}

#endif /* ENABLE_SERIAL_BRIDGE */
