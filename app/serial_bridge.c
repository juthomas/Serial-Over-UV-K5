/* Serial Bridge — UART <-> FSK / DTMF RF tunnel for UV-K5 (egzumer-based)
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
 * Runtime * cycles FSK / data / Morse. In data mode, F cycles the
 * C-rooted scales (TEL / CHR / MAJ / MIN / DOR / PEN / BLU). SIDE1
 * toggles FSK/data tempo, or Morse WPM (20…100). F toggles Morse 1T/2T.
 * XOR: data UART at idle (speaker off); analog FM during a QSO.
 */

#ifdef ENABLE_SERIAL_BRIDGE

#include "app/serial_bridge.h"

#include <string.h>

#include "app/app.h"
#include "app/generic.h"
#include "audio.h"
#include "bsp/dp32g030/dma.h"
#include "driver/bk4819.h"
#include "driver/crc.h"
#include "driver/gpio.h"
#include "driver/system.h"
#include "driver/uart.h"
#include "frequencies.h"
#include "functions.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"
#include "ui/inputbox.h"
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
#define SB_MODE_MORSE  2u
#define SB_MODE_COUNT  3u

#define SB_SCALE_TEL   0u
#define SB_SCALE_CHR   1u
#define SB_SCALE_MAJ   2u
#define SB_SCALE_MIN   3u
#define SB_SCALE_DOR   4u
#define SB_SCALE_PEN   5u
#define SB_SCALE_BLU   6u
#define SB_SCALE_COUNT 7u

#define SB_TONE1_GAIN  65u

#define SB_DTMF_TONE_TICKS    8u   /* TEL 80 ms */
#define SB_SCALE_TONE_TICKS  12u   /* musical 120 ms (~300 BPM) */
#define SB_FAST_TONE_TICKS    5u   /* 50 ms — SelCall / DTMF floor */
#define SB_FAST_GAP_TICKS     3u   /* 30 ms */
#define SB_DTMF_GAP_TICKS     8u   /* 80 ms */
#define SB_DTMF_KEYUP_TICKS  5u   /* 50 ms after PA up */
#define SB_DTMF_IDLE_TICKS   8u   /* 80 ms UART idle flush */
#define SB_DTMF_TX_GAP       40u  /* 400 ms ignore own echo */
#define SB_FAST_TX_GAP        8u  /* 80 ms echo window at 2K */
#define SB_FSK_REG72_1200  0x3065u
#define SB_FSK_REG72_2400  0x60CBu  /* Tone2 = 2400 Hz */
#define SB_FSK_REG58_1200  0x00C1u
#define SB_FSK_REG58_2400  0x01C1u  /* 2.4k RX bandwidth */
#define SB_DTMF_RX_TIMEOUT   40u  /* 400 ms without a symbol */
#define SB_DTMF_RECENT       30u  /* 300 ms: treat RX as data, not voice */
#define SB_DTMF_SYMS_MAX     22u  /* ** + LEN + SEQ + 16 data + 2 CRC */
#define SB_DTMF_XOFF_WATER   32u
#define SB_MORSE_XOFF_WATER  16u
#define SB_CW_HZ             600u
#define SB_CW_DIT_HZ         800u /* dual-tone: short */
#define SB_CW_DAH_HZ         600u /* dual-tone: long — in Goertzel band */
#define SB_CW_KEYUP_MS       50u
#define SB_CW_RX_HOLD_MS     400u
#define SB_CW_WPM_COUNT      6u
#define SB_CW_COEFF          115u /* round(128*cos(2π*600/8422)) */
#define SB_CW_DIT_COEFF      106u /* 800 Hz */
#define SB_CW_DAH_COEFF      115u /* 600 Hz */
#define SB_CW_DUMMY_COEFF     44u /* 1633 Hz — unused SelCall bins */

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

enum {
	SB_CW_TX_IDLE = 0,
	SB_CW_TX_KEYUP,
	SB_CW_TX_MARK,
	SB_CW_TX_ELEM_GAP,
	SB_CW_TX_LETTER_GAP,
	SB_CW_TX_WORD_GAP
};

static uint8_t  s_uart_rx[SB_UART_BUF];
static uint16_t s_uart_len;
static uint8_t  s_uart_dma_idx;
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
static uint8_t  s_scale = SB_SCALE_TEL;
static bool     s_tempo_fast;

/* Row 0 (TEL) unused — dual-tone DTMF. Others: 16 unique pitches in the
 * BK4819 Goertzel band. C4 (~262 Hz) is inaudible to SelCall and several
 * notes shared the same coeff (tie → no IRQ). CHR = 12-TET from G5. */
static const char * const s_scale_name[SB_SCALE_COUNT] = {
	"TEL", "CHR", "MAJ", "MIN", "DOR", "PEN", "BLU"
};

static const uint16_t s_scale_hz[SB_SCALE_COUNT][16] = {
	{ 0 },
	{ 784, 831, 880, 932,  988, 1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865 },
	{ 784, 838, 896, 945,  993, 1058, 1131, 1170, 1250, 1336, 1382, 1477, 1579, 1688, 1745, 1865 },
	{ 784, 838, 879, 929,  990, 1024, 1094, 1170, 1250, 1293, 1382, 1477, 1527, 1632, 1745, 1865 },
	{ 784, 838, 879, 929,  990, 1058, 1097, 1170, 1250, 1293, 1382, 1477, 1579, 1632, 1745, 1865 },
	{ 784, 826, 879, 929,  977, 1047, 1098, 1152, 1239, 1300, 1397, 1466, 1538, 1654, 1735, 1865 },
	{ 784, 855, 906, 945,  993, 1047, 1109, 1209, 1281, 1319, 1357, 1480, 1568, 1710, 1812, 1865 }
};

/* Stock DTMF / 5-tone Goertzel (BK4819_Init). */
static const uint8_t s_stock_goertzel[16] = {
	111, 107, 103, 98, 80, 71, 58, 44, 65, 55, 37, 23, 228, 203, 181, 159
};

/* 8-bit Goertzel: round(128 * cos(2π f / 8422)), fitted to 697→111 … 1633→44. */
static const uint8_t s_scale_coeff[SB_SCALE_COUNT][16] = {
	{ 0 },
	{ 107, 104, 101,  98,  95,  91,  87,  82,  77,  71,  65,  58,  50,  42,  33,  23 },
	{ 107, 104, 100,  97,  94,  90,  85,  82,  76,  70,  66,  58,  49,  39,  34,  23 },
	{ 107, 104, 101,  98,  95,  92,  88,  82,  76,  73,  66,  58,  54,  44,  34,  23 },
	{ 107, 104, 101,  98,  95,  90,  87,  82,  76,  73,  66,  58,  49,  44,  34,  23 },
	{ 107, 104, 101,  98,  95,  91,  87,  84,  77,  72,  65,  59,  53,  42,  35,  23 },
	{ 107, 103, 100,  97,  94,  91,  87,  79,  74,  71,  68,  58,  50,  37,  28,  23 }
};

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

static uint8_t  s_cw_tx_state;
static uint16_t s_cw_tx_ms;
static uint8_t  s_cw_tx_bytes;
static uint8_t  s_cw_tx_idx;
static uint8_t  s_cw_qidx;
static uint8_t  s_cw_qlen;
static char     s_cw_queue[3];
static uint8_t  s_cw_bits;
static uint8_t  s_cw_len;
static uint8_t  s_cw_elem;
static uint16_t s_cw_rx_on;
static uint16_t s_cw_rx_off;
static uint8_t  s_cw_rx_bits;
static uint8_t  s_cw_rx_len;
static uint8_t  s_cw_rx_esc;
static uint8_t  s_cw_rx_hex;
static uint16_t s_cw_rx_recent;
static uint8_t  s_cw_wpm_idx;
static bool     s_cw_dual_tone;
static bool     s_cw_rx_was_on;
static bool     s_cw_hit;
static bool     s_cw_rx_dah_vote;
static uint16_t s_cw_tone_ms;

/* PARIS: unit_ms = 1200 / WPM. 1 ms TX/RX so 100 WPM (12 ms) is reachable. */
static const uint8_t s_cw_unit_ms[SB_CW_WPM_COUNT] = { 60, 40, 30, 20, 15, 12 };
static const char * const s_cw_wpm_name[SB_CW_WPM_COUNT] = {
	"20", "30", "40", "60", "80", "100"
};
static const char * const s_cw_wpm_dual_name[SB_CW_WPM_COUNT] = {
	"20 2T", "30 2T", "40 2T", "60 2T", "80 2T", "100 2T"
};

uint16_t gSerialBridgeFSKBuffer[36];
uint16_t gSerialBridgeTxPackets;
uint16_t gSerialBridgeRxPackets;
uint16_t gSerialBridgeRxErrors;
uint16_t gSerialBridgeTxBytes;
uint16_t gSerialBridgeRxBytes;
bool     gSerialBridgeBusyTx;

static bool SB_IsToneAir(void)
{
	return s_mode == SB_MODE_DTMF || s_mode == SB_MODE_MORSE;
}

static uint8_t SB_PayloadMax(void)
{
	if (s_mode == SB_MODE_DTMF)
		return SERIAL_BRIDGE_DTMF_PAYLOAD_MAX;
	if (s_mode == SB_MODE_MORSE)
		return SERIAL_BRIDGE_MORSE_PAYLOAD_MAX;
	return SERIAL_BRIDGE_PAYLOAD_MAX;
}

static uint8_t SB_IdleFlushTicks(void)
{
	return SB_IsToneAir() ? SB_DTMF_IDLE_TICKS : SB_IDLE_TICKS;
}

static void SB_SetupFSK(void)
{
	/* FSK modem only. Do not enable Tone-2 (REG_70): it mutes analog FM RX.
	 * 2K = 2400 baud (Tone2 + wider RX BW); normal = 1200. */
	if (s_tempo_fast) {
		BK4819_WriteRegister(BK4819_REG_72, SB_FSK_REG72_2400);
		BK4819_WriteRegister(BK4819_REG_58, SB_FSK_REG58_2400);
	} else {
		BK4819_WriteRegister(BK4819_REG_72, SB_FSK_REG72_1200);
		BK4819_WriteRegister(BK4819_REG_58, SB_FSK_REG58_1200);
	}
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
	const uint16_t dma_len = DMA_CH0->ST & 0xFFFU;
	const bool drop_noise = gSerialBridgeBusyTx || s_tx_gap > 0;

	while (s_uart_dma_idx != dma_len) {
		const uint8_t b = UART_DMA_Buffer[s_uart_dma_idx];

		/* EMI during TX is 0xFF (CH340) or 0x00 (PL2303). */
		if (drop_noise && (b == 0x00u || b == 0xFFu)) {
			s_uart_dma_idx = (uint8_t)((s_uart_dma_idx + 1u) % sizeof(UART_DMA_Buffer));
			continue;
		}
		if (s_uart_len >= SB_UART_BUF) {
			SB_UartXoff();
			break;
		}
		s_uart_rx[s_uart_len++] = b;
		s_uart_dma_idx = (uint8_t)((s_uart_dma_idx + 1u) % sizeof(UART_DMA_Buffer));
		s_idle_ticks = 0;
		if ((s_mode == SB_MODE_DTMF && s_uart_len >= SB_DTMF_XOFF_WATER)
		    || (s_mode == SB_MODE_MORSE && s_uart_len >= SB_MORSE_XOFF_WATER))
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
	gTxVfo->CHANNEL_SAVE = (uint8_t)(FREQ_CHANNEL_FIRST + band);
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

/* Same rounding as driver/bk4819.c scale_freq() (not exported). */
static uint16_t SB_ToneWord(uint16_t hz)
{
	return (uint16_t)((((uint32_t)hz * 1353245u) + (1u << 16)) >> 17);
}

static void SB_WriteGoertzel(const uint8_t *coeff)
{
	uint8_t i;

	for (i = 0; i < 16; i++)
		BK4819_WriteRegister(BK4819_REG_09, ((uint16_t)i << 12) | coeff[i]);
}

static void SB_EnableDtmfDetect(void)
{
	SB_WriteGoertzel(s_stock_goertzel);
	BK4819_EnableDTMF();
}

static void SB_EnableSelCallDetect(void)
{
	SB_WriteGoertzel(s_scale_coeff[s_scale]);
	BK4819_WriteRegister(BK4819_REG_21, 0x06D8);
	BK4819_WriteRegister(BK4819_REG_24,
		(1u   << BK4819_REG_24_SHIFT_UNKNOWN_15) |
		(80u  << BK4819_REG_24_SHIFT_THRESHOLD)  |
		(1u   << BK4819_REG_24_SHIFT_UNKNOWN_6)  |
		        BK4819_REG_24_ENABLE             |
		        BK4819_REG_24_SELECT_SELCALL     |
		(15u  << BK4819_REG_24_SHIFT_MAX_SYMBOLS));
}

static uint16_t SB_CwReg24(bool enable)
{
	uint16_t v =
		(1u  << BK4819_REG_24_SHIFT_UNKNOWN_15) |
		(80u << BK4819_REG_24_SHIFT_THRESHOLD)  |
		(1u  << BK4819_REG_24_SHIFT_UNKNOWN_6)  |
		       BK4819_REG_24_SELECT_SELCALL     |
		(1u  << BK4819_REG_24_SHIFT_MAX_SYMBOLS);

	if (enable)
		v |= BK4819_REG_24_ENABLE;
	return v;
}

static void SB_EnableCwDetect(void)
{
	uint8_t coeff[16];
	uint8_t i;

	/* Always 800/600 bins. 16 identical 600 Hz coeffs = SelCall tie (no IRQ).
	 * One pulse per mark cannot carry duration — 1T stretched every hit to
	 * 1 unit and decoded only E/I/S/H. */
	for (i = 0; i < 16; i++)
		coeff[i] = SB_CW_DUMMY_COEFF;
	coeff[0] = SB_CW_DIT_COEFF;
	coeff[1] = SB_CW_DAH_COEFF;
	SB_WriteGoertzel(coeff);
	BK4819_WriteRegister(BK4819_REG_21, 0x06D8);
	BK4819_WriteRegister(BK4819_REG_24, SB_CwReg24(false));
	BK4819_WriteRegister(BK4819_REG_24, SB_CwReg24(true));
}

static void SB_ScaleApply(void)
{
	if (s_mode == SB_MODE_MORSE)
		SB_EnableCwDetect();
	else if (s_mode == SB_MODE_DTMF && s_scale != SB_SCALE_TEL)
		SB_EnableSelCallDetect();
	else
		SB_EnableDtmfDetect();
}

void SERIAL_BRIDGE_ApplyDecoder(void)
{
	if (!s_active)
		return;
	SB_ScaleApply();
}

/* ITU Morse: bit0 = first element, 0 = dit, 1 = dah. '=' is decode-only (escape). */
static const char s_cw_itu_char[] = {
	'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T',
	'U','V','W','X','Y','Z','0','1','2','3','4','5','6','7','8','9',
	'.',',','?','\'','!','/','(',')','&',':',';','=','+','-','_','"','$','@'
};
static const uint8_t s_cw_itu_bits[] = {
	2, 1, 5, 1, 0, 4, 3, 0, 0, 14, 5, 2, 3, 1, 7, 6, 11, 2, 0, 1,
	4, 8, 6, 9, 13, 3, 31, 30, 28, 24, 16, 0, 1, 3, 7, 15,
	42, 51, 12, 30, 53, 9, 13, 45, 2, 7, 21, 17, 10, 33, 44, 18, 72, 22
};
static const uint8_t s_cw_itu_nlen[] = {
	2, 4, 4, 3, 1, 4, 3, 4, 2, 4, 3, 4, 2, 2, 3, 4, 4, 3, 3, 1,
	3, 4, 3, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
	6, 6, 6, 6, 6, 5, 5, 6, 5, 6, 6, 5, 5, 6, 6, 6, 7, 6
};

static uint16_t SB_CwUnitMs(void)
{
	uint8_t i = s_cw_wpm_idx;

	if (i >= SB_CW_WPM_COUNT)
		i = 0;
	return s_cw_unit_ms[i];
}

static bool SB_CwLookup(char c, uint8_t *bits, uint8_t *len)
{
	uint8_t i;

	for (i = 0; i < ARRAY_SIZE(s_cw_itu_char); i++) {
		if (s_cw_itu_char[i] == c) {
			*bits = s_cw_itu_bits[i];
			*len  = s_cw_itu_nlen[i];
			return true;
		}
	}
	return false;
}

static char SB_CwHex(uint8_t n)
{
	n &= 0x0Fu;
	return (n < 10) ? (char)('0' + n) : (char)('A' + (n - 10));
}

static int8_t SB_CwHexVal(char c)
{
	if (c >= '0' && c <= '9')
		return (int8_t)(c - '0');
	if (c >= 'A' && c <= 'F')
		return (int8_t)(c - 'A' + 10);
	return -1;
}

static void SB_CwQueueByte(uint8_t b)
{
	s_cw_qidx = 0;
	if (b >= 'a' && b <= 'z')
		b = (uint8_t)(b - 32u);
	if (b == '\r' || b == '\n' || b == '\t')
		b = ' ';

	if (b == ' ') {
		s_cw_queue[0] = ' ';
		s_cw_qlen = 1;
		return;
	}
	if (b != '=' && SB_CwLookup((char)b, &s_cw_bits, &s_cw_len)) {
		s_cw_queue[0] = (char)b;
		s_cw_qlen = 1;
		return;
	}
	s_cw_queue[0] = '=';
	s_cw_queue[1] = SB_CwHex((uint8_t)(b >> 4));
	s_cw_queue[2] = SB_CwHex(b);
	s_cw_qlen = 3;
}

static void SB_CwLoadQueued(void)
{
	char c = s_cw_queue[s_cw_qidx];

	s_cw_elem = 0;
	if (c == ' ') {
		s_cw_len = 0;
		return;
	}
	if (!SB_CwLookup(c, &s_cw_bits, &s_cw_len)) {
		s_cw_len = 0;
	}
}

static uint16_t SB_CwMarkHz(bool dah)
{
	/* Pitch encodes dit/dah; 1T (same 600 Hz) cannot be timed from a pulse IRQ. */
	return dah ? SB_CW_DAH_HZ : SB_CW_DIT_HZ;
}

static void SB_CwTonePrep(bool dah)
{
	BK4819_WriteRegister(BK4819_REG_70,
		BK4819_REG_70_ENABLE_TONE1 |
		(SB_TONE1_GAIN << BK4819_REG_70_SHIFT_TONE1_TUNING_GAIN));
	BK4819_WriteRegister(BK4819_REG_71, SB_ToneWord(SB_CwMarkHz(dah)));
	BK4819_WriteRegister(BK4819_REG_51, 0x0000);
}

static void SB_CwTxAbort(void)
{
	if (s_cw_tx_state == SB_CW_TX_IDLE)
		return;

	BK4819_EnterTxMute();
	BK4819_WriteRegister(BK4819_REG_70, 0x0000);
	BK4819_SetupPowerAmplifier(0, 0);
	BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, false);
	BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);
	s_cw_tx_state = SB_CW_TX_IDLE;
	s_cw_tx_bytes = 0;
	gSerialBridgeBusyTx = false;
}

static void SB_CwTxFinish(void)
{
	BK4819_EnterTxMute();
	BK4819_WriteRegister(BK4819_REG_70, 0x0000);
	BK4819_SetupPowerAmplifier(0, 0);
	BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, false);
	BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);

	gSerialBridgeTxPackets++;
	gSerialBridgeTxBytes += s_cw_tx_bytes;
	SB_ConsumeUart(s_cw_tx_bytes);

	s_cw_tx_state = SB_CW_TX_IDLE;
	s_cw_tx_bytes = 0;
	{
		uint16_t gap = (uint16_t)((SB_CwUnitMs() * 7u + 9u) / 10u);

		s_tx_gap = (gap > 255u) ? 255u : (uint8_t)gap;
	}
	s_pending_rearm = true;
	gSerialBridgeBusyTx = true;

	SB_DrainUart();
	RADIO_SetupRegisters(true);
	gUpdateDisplay = true;
}

static bool SB_CwStartNextChar(void)
{
	while (s_cw_qidx >= s_cw_qlen) {
		s_cw_tx_idx++;
		if (s_cw_tx_idx >= s_cw_tx_bytes)
			return false;
		SB_CwQueueByte(s_uart_rx[s_cw_tx_idx]);
	}
	SB_CwLoadQueued();
	s_cw_qidx++;
	return true;
}

static void SB_CwTxStart(uint8_t len)
{
	if (len == 0)
		return;

	s_cw_tx_bytes = len;
	s_cw_tx_idx   = 0;
	SB_CwQueueByte(s_uart_rx[0]);
	s_cw_tx_ms    = SB_CW_KEYUP_MS;
	s_cw_tx_state = SB_CW_TX_KEYUP;

	gSerialBridgeBusyTx = true;
	SB_UartXoff();
	BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);
	RADIO_SetTxParameters();
	BK4819_EnterTxMute();
	BK4819_SetAF(BK4819_AF_MUTE);
	SB_CwTonePrep(false);
	BK4819_EnableTXLink();
}

static void SB_CwBeginMark(void)
{
	const uint16_t unit = SB_CwUnitMs();
	const bool dah = (s_cw_bits & (1u << s_cw_elem)) != 0;

	SB_CwTonePrep(dah);
	BK4819_ExitTxMute();
	s_cw_tx_state = SB_CW_TX_MARK;
	s_cw_tx_ms = (uint16_t)(unit * (dah ? 3u : 1u));
}

static void SB_CwTxTick(void)
{
	const uint16_t unit = SB_CwUnitMs();

	if (s_cw_tx_ms > 0) {
		s_cw_tx_ms--;
		return;
	}

	switch (s_cw_tx_state) {
	case SB_CW_TX_KEYUP:
		if (!SB_CwStartNextChar()) {
			SB_CwTxFinish();
			break;
		}
		if (s_cw_len == 0) {
			s_cw_tx_state = SB_CW_TX_WORD_GAP;
			s_cw_tx_ms = (uint16_t)(unit * 7u);
			break;
		}
		SB_CwBeginMark();
		break;

	case SB_CW_TX_MARK:
		BK4819_EnterTxMute();
		s_cw_elem++;
		s_cw_tx_state = SB_CW_TX_ELEM_GAP;
		s_cw_tx_ms = unit;
		break;

	case SB_CW_TX_ELEM_GAP:
		if (s_cw_elem < s_cw_len) {
			SB_CwBeginMark();
			break;
		}
		s_cw_tx_state = SB_CW_TX_LETTER_GAP;
		s_cw_tx_ms = (uint16_t)(unit * 2u);
		break;

	case SB_CW_TX_LETTER_GAP:
		if (!SB_CwStartNextChar()) {
			s_cw_tx_state = SB_CW_TX_WORD_GAP;
			s_cw_tx_ms = (uint16_t)(unit * 4u);
			break;
		}
		if (s_cw_len == 0) {
			s_cw_tx_state = SB_CW_TX_WORD_GAP;
			s_cw_tx_ms = (uint16_t)(unit * 4u);
			break;
		}
		SB_CwBeginMark();
		break;

	case SB_CW_TX_WORD_GAP:
		if (s_cw_tx_idx >= s_cw_tx_bytes) {
			SB_CwTxFinish();
			break;
		}
		if (!SB_CwStartNextChar()) {
			SB_CwTxFinish();
			break;
		}
		if (s_cw_len == 0) {
			s_cw_tx_ms = (uint16_t)(unit * 7u);
			break;
		}
		SB_CwBeginMark();
		break;

	default:
		SB_CwTxFinish();
		break;
	}
}

static void SB_CwTxService(void)
{
	uint8_t drain = 0;

	while (s_cw_tx_state != SB_CW_TX_IDLE) {
		if (SB_AnalogBusy()) {
			SB_CwTxAbort();
			return;
		}
		if (s_cw_tx_ms > 0) {
			SYSTEM_DelayMs(1);
			if (++drain >= 10) {
				SB_DrainUart();
				drain = 0;
			}
		}
		SB_CwTxTick();
	}
	SB_DrainUart();
}

static void SB_CwRxReset(void)
{
	s_cw_rx_on = 0;
	s_cw_rx_off = 0;
	s_cw_rx_bits = 0;
	s_cw_rx_len = 0;
	s_cw_rx_esc = 0;
	s_cw_rx_hex = 0;
	s_cw_rx_was_on = false;
	s_cw_hit = false;
	s_cw_rx_dah_vote = false;
	s_cw_tone_ms = 0;
}

static void SB_CwEmit(char c)
{
	int8_t hv;

	if (s_cw_rx_esc == 0) {
		if (c == '=') {
			s_cw_rx_esc = 1;
			return;
		}
	} else if (s_cw_rx_esc == 1) {
		hv = SB_CwHexVal(c);
		if (hv < 0) {
			s_cw_rx_esc = 0;
			return;
		}
		s_cw_rx_hex = (uint8_t)hv;
		s_cw_rx_esc = 2;
		return;
	} else {
		hv = SB_CwHexVal(c);
		s_cw_rx_esc = 0;
		if (hv < 0)
			return;
		c = (char)((s_cw_rx_hex << 4) | (uint8_t)hv);
	}

	UART_Send((uint8_t *)&c, 1);
	gSerialBridgeRxPackets++;
	gSerialBridgeRxBytes++;
	gUpdateDisplay = true;
}

static void SB_CwRxEndLetter(void)
{
	uint8_t i;
	char ch = 0;

	if (s_cw_rx_len == 0)
		return;

	for (i = 0; i < ARRAY_SIZE(s_cw_itu_char); i++) {
		if (s_cw_itu_nlen[i] == s_cw_rx_len && s_cw_itu_bits[i] == s_cw_rx_bits) {
			ch = s_cw_itu_char[i];
			break;
		}
	}
	s_cw_rx_bits = 0;
	s_cw_rx_len = 0;
	if (ch != 0)
		SB_CwEmit(ch);
}

static void SB_CwRxTick(uint8_t dt_ms)
{
	const uint16_t unit = SB_CwUnitMs();
	const uint16_t word = (uint16_t)(unit * 7u);
	const uint16_t letter = (uint16_t)(unit * 2u);
	bool on;
	uint16_t hold;

	/* Glue successive IRQs of the same mark. Must stay < 1 unit or dits merge. */
	hold = unit / 3u;
	if (hold < dt_ms)
		hold = dt_ms;
	if (s_cw_hit) {
		s_cw_hit = false;
		s_cw_tone_ms = hold;
	}

	on = s_cw_tone_ms > 0;
	if (s_cw_tone_ms > dt_ms)
		s_cw_tone_ms -= dt_ms;
	else
		s_cw_tone_ms = 0;

	if (s_cw_rx_recent > dt_ms)
		s_cw_rx_recent -= dt_ms;
	else
		s_cw_rx_recent = 0;

	if (on) {
		s_cw_rx_recent = SB_CW_RX_HOLD_MS;
		if (!s_cw_rx_was_on) {
			if (s_cw_rx_off >= (uint16_t)(unit * 5u)) {
				SB_CwRxEndLetter();
				if (s_cw_rx_esc == 0)
					SB_CwEmit(' ');
			} else if (s_cw_rx_off >= letter) {
				SB_CwRxEndLetter();
			}
			s_cw_rx_on = dt_ms;
		} else if (s_cw_rx_on < 2000u - dt_ms)
			s_cw_rx_on += dt_ms;
		s_cw_rx_off = 0;
		s_cw_rx_was_on = true;
	} else {
		if (s_cw_rx_was_on) {
			if (s_cw_rx_len < 7) {
				if (s_cw_rx_dah_vote)
					s_cw_rx_bits |= (uint8_t)(1u << s_cw_rx_len);
				s_cw_rx_len++;
			}
			s_cw_rx_dah_vote = false;
			s_cw_rx_off = dt_ms;
		} else if (s_cw_rx_off < 2000u - dt_ms) {
			const uint16_t prev = s_cw_rx_off;

			s_cw_rx_off += dt_ms;
			if (prev < word && s_cw_rx_off >= word) {
				SB_CwRxEndLetter();
				if (s_cw_rx_esc == 0)
					SB_CwEmit(' ');
			}
		}
		s_cw_rx_was_on = false;
	}
}

static void SB_CwRxService(void)
{
	SB_CwRxTick(10);
}

void SERIAL_BRIDGE_OnCwHit(uint8_t code)
{
	if (!s_active || s_mode != SB_MODE_MORSE
	    || s_cw_tx_state != SB_CW_TX_IDLE
	    || gSerialBridgeBusyTx)
		return;

	s_cw_hit = true;
	if (code == 1u)
		s_cw_rx_dah_vote = true;

	/* Retrigger: a long dah must keep producing hits if the chip latches. */
	BK4819_WriteRegister(BK4819_REG_24, SB_CwReg24(false));
	BK4819_WriteRegister(BK4819_REG_24, SB_CwReg24(true));
}

static uint8_t SB_ToneTicks(void)
{
	if (s_tempo_fast)
		return SB_FAST_TONE_TICKS;
	return (s_scale == SB_SCALE_TEL) ? SB_DTMF_TONE_TICKS : SB_SCALE_TONE_TICKS;
}

static uint8_t SB_GapTicks(void)
{
	if (s_tempo_fast)
		return SB_FAST_GAP_TICKS;
	return SB_DTMF_GAP_TICKS;
}

static void SB_PlaySymbol(char c)
{
	int8_t n;

	if (s_scale == SB_SCALE_TEL) {
		BK4819_PlayDTMF(c);
		return;
	}

	n = SB_SymToNibble(c);
	if (n < 0)
		return;

	BK4819_WriteRegister(BK4819_REG_70,
		BK4819_REG_70_ENABLE_TONE1 |
		(SB_TONE1_GAIN << BK4819_REG_70_SHIFT_TONE1_TUNING_GAIN));
	BK4819_WriteRegister(BK4819_REG_71, SB_ToneWord(s_scale_hz[s_scale][n]));
	BK4819_WriteRegister(BK4819_REG_51, 0x0000);
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
	s_tx_gap = s_tempo_fast ? SB_FAST_TX_GAP : SB_DTMF_TX_GAP;
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
	if (s_scale == SB_SCALE_TEL) {
		BK4819_EnterDTMF_TX(false);
	} else {
		/* Single TONE1 — do not EnableDTMF() (that restores phone coeffs). */
		BK4819_EnterTxMute();
		BK4819_SetAF(BK4819_AF_MUTE);
		BK4819_WriteRegister(BK4819_REG_70,
			BK4819_REG_70_ENABLE_TONE1 |
			(SB_TONE1_GAIN << BK4819_REG_70_SHIFT_TONE1_TUNING_GAIN));
		BK4819_WriteRegister(BK4819_REG_51, 0x0000);
		BK4819_EnableTXLink();
	}
}

static void SB_DtmfTxTick(void)
{
	if (s_dtmf_tx_ticks > 0) {
		s_dtmf_tx_ticks--;
		return;
	}

	switch (s_dtmf_tx_state) {
	case SB_DTMF_TX_KEYUP:
		SB_PlaySymbol(s_dtmf_tx_syms[s_dtmf_tx_idx]);
		BK4819_ExitTxMute();
		s_dtmf_tx_state = SB_DTMF_TX_SYM;
		s_dtmf_tx_ticks = SB_ToneTicks();
		break;

	case SB_DTMF_TX_SYM:
		BK4819_EnterTxMute();
		s_dtmf_tx_idx++;
		s_dtmf_tx_state = SB_DTMF_TX_PAUSE;
		s_dtmf_tx_ticks = SB_GapTicks();
		break;

	case SB_DTMF_TX_PAUSE:
		if (s_dtmf_tx_idx >= s_dtmf_tx_nsyms) {
			SB_DtmfTxFinish();
			break;
		}
		SB_PlaySymbol(s_dtmf_tx_syms[s_dtmf_tx_idx]);
		BK4819_ExitTxMute();
		s_dtmf_tx_state = SB_DTMF_TX_SYM;
		s_dtmf_tx_ticks = SB_ToneTicks();
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

static void SB_FeedNibble(int8_t n)
{
	if (n < 0 || n > 15)
		return;
	if (gSerialBridgeBusyTx || s_tx_gap > 0 || SB_AnalogBusy())
		return;

	s_dtmf_rx_recent = SB_DTMF_RECENT;
	s_dtmf_rx_age    = 0;
	SB_SpeakerOff();

	switch (s_dtmf_rx_state) {
	case SB_DTMF_RX_SYNC0:
		if (n == 14)
			s_dtmf_rx_state = SB_DTMF_RX_SYNC1;
		break;

	case SB_DTMF_RX_SYNC1:
		s_dtmf_rx_state = (n == 14) ? SB_DTMF_RX_LEN : SB_DTMF_RX_SYNC0;
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

void SERIAL_BRIDGE_OnDtmf(char c)
{
	if (!s_active || s_mode != SB_MODE_DTMF)
		return;
	SB_FeedNibble(SB_SymToNibble(c));
}

void SERIAL_BRIDGE_OnSymbol(uint8_t code)
{
	if (!s_active || s_mode != SB_MODE_DTMF)
		return;
	SB_FeedNibble((int8_t)(code & 0x0Fu));
}

static void SB_ToggleMode(void)
{
	if (s_dtmf_tx_state != SB_DTMF_TX_IDLE)
		SB_DtmfTxAbort();
	SB_CwTxAbort();

	SB_DtmfRxReset();
	SB_CwRxReset();
	s_dtmf_rx_recent = 0;
	s_have_rx_seq = false;
	gFSKWriteIndex = 0;
	s_mode++;
	if (s_mode >= SB_MODE_COUNT)
		s_mode = SB_MODE_FSK;
	SB_ScaleApply();

	BK4819_WriteRegister(BK4819_REG_58, 0x0000);
	BK4819_WriteRegister(BK4819_REG_70, 0x0000);
	BK4819_WriteRegister(BK4819_REG_59, 0x0068);
	s_pending_rearm = true;
	SERIAL_BRIDGE_ReArm();

	gBeepToPlay = BEEP_1KHZ_60MS_OPTIONAL;
	gUpdateDisplay = true;
}

static void SB_CycleScale(void)
{
	if (s_mode == SB_MODE_MORSE) {
		if (s_cw_tx_state != SB_CW_TX_IDLE)
			return;
		s_cw_dual_tone = !s_cw_dual_tone;
		SB_ScaleApply();
		gBeepToPlay = BEEP_1KHZ_60MS_OPTIONAL;
		gUpdateDisplay = true;
		return;
	}

	if (s_mode != SB_MODE_DTMF || s_dtmf_tx_state != SB_DTMF_TX_IDLE)
		return;

	s_scale++;
	if (s_scale >= SB_SCALE_COUNT)
		s_scale = SB_SCALE_TEL;

	SB_DtmfRxReset();
	s_dtmf_rx_recent = 0;
	s_have_rx_seq = false;
	SB_ScaleApply();

	gBeepToPlay = BEEP_1KHZ_60MS_OPTIONAL;
	gUpdateDisplay = true;
}

static void SB_CycleTempo(void)
{
	if (s_dtmf_tx_state != SB_DTMF_TX_IDLE || s_cw_tx_state != SB_CW_TX_IDLE)
		return;

	if (s_mode == SB_MODE_MORSE) {
		s_cw_wpm_idx++;
		if (s_cw_wpm_idx >= SB_CW_WPM_COUNT)
			s_cw_wpm_idx = 0;
	} else {
		s_tempo_fast = !s_tempo_fast;
		if (s_mode == SB_MODE_FSK)
			SERIAL_BRIDGE_ReArm();
	}
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

	if (s_tx_gap > 0)
		return;

	/* FSK bursts open squelch — still (re)enable the modem.
	 * Data/TEL keep the old gate so analog QSO can take the speaker. */
	if (SB_IsToneAir()) {
		if (SB_AnalogRxActive()
		    || g_SquelchLost
		    || gEndOfRxDetectedMaybe
		    || gTailToneEliminationCountdown_10ms > 0)
			return;
	}

	s_pending_rearm = false;
	gFSKWriteIndex = 0;

	if (SB_IsToneAir()) {
		BK4819_WriteRegister(BK4819_REG_58, 0x0000);
		BK4819_WriteRegister(BK4819_REG_70, 0x0000);
		SB_ScaleApply();
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
	SB_CwTxAbort();

	gSerialBridgeBusyTx = false;
	gFSKWriteIndex = 0;
	s_pending_rearm = true;

	/* FSK (REG_58) and Tone-2 (REG_70) steal analog MIC deviation. */
	BK4819_WriteRegister(BK4819_REG_58, 0x0000);
	BK4819_WriteRegister(BK4819_REG_70, 0x0000);
	BK4819_WriteRegister(BK4819_REG_59, 0x0068);
	BK4819_WriteRegister(BK4819_REG_7D, 0xE940 | (gEeprom.MIC_SENSITIVITY_TUNING & 0x1f));
	SB_ScaleApply();
}

void SERIAL_BRIDGE_EnterAnalogRx(void)
{
	if (!s_active)
		return;

	s_pending_rearm = true;
	gFSKWriteIndex = 0;
	SB_DtmfRxReset();

	/* Disable FSK modem only. Do not write REG_59=0x0068: that mutes AF on V1.
	 * Keep the current data decoder (SelCall / TEL); do not force phone DTMF. */
	BK4819_WriteRegister(BK4819_REG_58, 0x0000);
	BK4819_WriteRegister(BK4819_REG_70, 0x0000);
	SB_ScaleApply();
	BK4819_RX_TurnOn();
}

void SERIAL_BRIDGE_HoldAfterTx(void)
{
	if (!s_active)
		return;

	if (SB_IsToneAir())
		s_tx_gap = s_tempo_fast ? SB_FAST_TX_GAP : SB_DTMF_TX_GAP;
	else
		s_tx_gap = s_tempo_fast ? (SB_TX_GAP / 2u) : SB_TX_GAP;
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
	s_tx_gap = s_tempo_fast ? (SB_TX_GAP / 2u) : SB_TX_GAP;
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

bool SERIAL_BRIDGE_IsFsk(void)
{
	return s_active && s_mode == SB_MODE_FSK;
}

bool SERIAL_BRIDGE_IsDtmf(void)
{
	return s_active && s_mode == SB_MODE_DTMF;
}

bool SERIAL_BRIDGE_IsMorse(void)
{
	return s_active && s_mode == SB_MODE_MORSE;
}

bool SERIAL_BRIDGE_UsesSelCall(void)
{
	return s_active && s_mode == SB_MODE_DTMF && s_scale != SB_SCALE_TEL;
}

const char *SERIAL_BRIDGE_ModeName(void)
{
	if (s_mode == SB_MODE_FSK)
		return "FSK";
	if (s_mode == SB_MODE_MORSE)
		return "CW";
	return SERIAL_BRIDGE_ScaleName();
}

const char *SERIAL_BRIDGE_NextModeName(void)
{
	if (s_mode == SB_MODE_FSK)
		return SERIAL_BRIDGE_ScaleName();
	if (s_mode == SB_MODE_DTMF)
		return "CW";
	return "FSK";
}

const char *SERIAL_BRIDGE_ScaleName(void)
{
	return s_scale_name[(s_scale < SB_SCALE_COUNT) ? s_scale : SB_SCALE_TEL];
}

const char *SERIAL_BRIDGE_NextScaleName(void)
{
	uint8_t next = (uint8_t)(s_scale + 1u);

	if (next >= SB_SCALE_COUNT)
		next = SB_SCALE_TEL;
	return s_scale_name[next];
}

const char *SERIAL_BRIDGE_NextCwToneName(void)
{
	return s_cw_dual_tone ? "1T" : "2T";
}

const char *SERIAL_BRIDGE_TempoLabel(void)
{
	if (s_mode == SB_MODE_MORSE) {
		uint8_t i = s_cw_wpm_idx;

		if (i >= SB_CW_WPM_COUNT)
			i = 0;
		return s_cw_dual_tone ? s_cw_wpm_dual_name[i] : s_cw_wpm_name[i];
	}
	return s_tempo_fast ? "2K" : "";
}

bool SERIAL_BRIDGE_HoldDataRx(void)
{
	if (!s_active)
		return false;
	if (s_mode == SB_MODE_MORSE)
		return g_SquelchLost || s_cw_rx_recent > 0 || s_cw_tx_state != SB_CW_TX_IDLE;
	if (s_mode != SB_MODE_DTMF)
		return false;
	return g_SquelchLost
	    || s_dtmf_rx_recent > 0
	    || s_dtmf_rx_state != SB_DTMF_RX_SYNC0;
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
	s_uart_dma_idx = (uint8_t)(DMA_CH0->ST & 0xFFFU);
	s_dtmf_tx_state = SB_DTMF_TX_IDLE;
	s_cw_tx_state  = SB_CW_TX_IDLE;
	s_cw_wpm_idx   = 0;
	s_cw_dual_tone = true;
	s_tempo_fast   = false;
	SB_DtmfRxReset();
	SB_CwRxReset();
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
	SB_ApplyFrequency(SERIAL_BRIDGE_DEFAULT_FREQ);
	gUpdateStatus  = true;
}

void SERIAL_BRIDGE_Stop(void)
{
	if (s_dtmf_tx_state != SB_DTMF_TX_IDLE)
		SB_DtmfTxAbort();
	SB_CwTxAbort();

	s_active = false;
	gSerialBridgeBusyTx = false;
	s_pending_rearm = false;
	gInputBoxIndex = 0;
	gEeprom.DUAL_WATCH = s_saved_dual_watch;
	gEeprom.CROSS_BAND_RX_TX = s_saved_cross_band;
	SB_EnableDtmfDetect();
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

	if (SB_IsToneAir())
		return;

	if (gFSKWriteIndex < 36)
		return;

	gFSKWriteIndex = 0;
	status = BK4819_ReadRegister(BK4819_REG_0B);

	if (SB_AnalogBusy())
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
	/* Own 600 Hz sidetone would decode as UART echo (no CRC like data). */
	if (s_mode == SB_MODE_MORSE
	    && s_cw_tx_state == SB_CW_TX_IDLE
	    && !gSerialBridgeBusyTx
	    && s_tx_gap == 0)
		SB_CwRxService();

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

	if (s_mode == SB_MODE_MORSE && s_cw_tx_state != SB_CW_TX_IDLE) {
		SB_CwTxService();
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

	if (SB_AnalogRxActive() && SB_IsToneAir()) {
		if (SERIAL_BRIDGE_HoldDataRx())
			SB_SpeakerOff();
		SB_DrainUart();
		return;
	}

	if (s_mode == SB_MODE_FSK || (!g_SquelchLost && gEnableSpeaker))
		SB_SpeakerOff();

	if (s_pending_rearm) {
		if (s_mode == SB_MODE_FSK
		    || (!g_SquelchLost
		        && !gEndOfRxDetectedMaybe
		        && gTailToneEliminationCountdown_10ms == 0))
			SERIAL_BRIDGE_ReArm();
	}

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
	else if (s_mode == SB_MODE_MORSE)
		SB_CwTxStart(chunk);
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

	/* SIDE1/SIDE2 fire on release, same as ACTION_Handle (lampe / moniteur). */
	if ((Key == KEY_SIDE1 || Key == KEY_SIDE2) && !bKeyHeld && !bKeyPressed) {
		if (!gPttIsPressed)
			SB_CycleTempo();
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

	case KEY_F:
		if (gInputBoxIndex == 0)
			SB_CycleScale();
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
