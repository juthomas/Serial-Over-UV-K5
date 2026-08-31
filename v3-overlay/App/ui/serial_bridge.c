#ifdef ENABLE_SERIAL_BRIDGE

#include <string.h>

#include "app/serial_bridge.h"
#include "driver/st7565.h"
#include "external/printf/printf.h"
#include "misc.h"
#include "radio.h"
#include "ui/helper.h"
#include "ui/inputbox.h"
#include "ui/serial_bridge.h"

void UI_DisplaySerialBridge(void)
{
	char String[22];

	UI_DisplayClear();

	{
		const char *tempo = SERIAL_BRIDGE_TempoLabel();
		const char *mode  = SERIAL_BRIDGE_ModeName();

		if (tempo[0] != '\0')
			sprintf(String, "SER %s %s", mode, tempo);
		else
			sprintf(String, "SER %s", mode);
		UI_PrintString(String, 2, 127, 0, 8);
	}

	if (gInputBoxIndex == 0) {
		uint32_t frequency = gTxVfo->freq_config_RX.Frequency;
		sprintf(String, "%3u.%05u", frequency / 100000, frequency % 100000);
		UI_PrintStringSmallNormal(String + 7, 97, 0, 3);
		String[7] = 0;
		UI_DisplayFrequency(String, 16, 2, false);
	} else {
		const char *ascii = INPUTBOX_GetAscii();
		sprintf(String, "%.3s.%.3s", ascii, ascii + 3);
		UI_DisplayFrequency(String, 16, 2, false);
	}

	if (gSerialBridgeBusyTx)
		sprintf(String, "TX %u B:%u", gSerialBridgeTxPackets, gSerialBridgeTxBytes);
	else
		sprintf(String, "RX %u B:%u", gSerialBridgeRxPackets, gSerialBridgeRxBytes);
	UI_PrintStringSmallNormal(String, 2, 127, 4);

	if (SERIAL_BRIDGE_IsDtmf())
		sprintf(String, "E:%u F=%s *=%s", gSerialBridgeRxErrors,
			SERIAL_BRIDGE_NextScaleName(), SERIAL_BRIDGE_NextModeName());
	else if (SERIAL_BRIDGE_IsMorse())
		sprintf(String, "E:%u F=%s *=%s", gSerialBridgeRxErrors,
			SERIAL_BRIDGE_NextCwToneName(), SERIAL_BRIDGE_NextModeName());
	else
		sprintf(String, "E:%u *=%s PTT", gSerialBridgeRxErrors,
			SERIAL_BRIDGE_NextModeName());
	UI_PrintStringSmallNormal(String, 2, 127, 5);

	ST7565_BlitFullScreen();
}

#endif
