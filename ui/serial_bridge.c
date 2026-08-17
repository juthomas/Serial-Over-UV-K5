#ifdef ENABLE_SERIAL_BRIDGE

#include <string.h>

#include "app/serial_bridge.h"
#include "driver/st7565.h"
#include "external/printf/printf.h"
#include "misc.h"
#include "radio.h"
#include "ui/helper.h"
#include "ui/serial_bridge.h"

void UI_DisplaySerialBridge(void)
{
	char String[22];
	uint32_t frequency;

	UI_DisplayClear();

	UI_PrintString("SER BRIDGE", 2, 127, 0, 8);

	frequency = gRxVfo->freq_config_RX.Frequency;
	sprintf(String, "%3u.%05u", frequency / 100000, frequency % 100000);
	UI_PrintStringSmallNormal(String + 7, 97, 0, 3);
	String[7] = 0;
	UI_DisplayFrequency(String, 16, 2, false);

	if (gSerialBridgeBusyTx)
		sprintf(String, "TX %u B:%u", gSerialBridgeTxPackets, gSerialBridgeTxBytes);
	else
		sprintf(String, "RX %u B:%u", gSerialBridgeRxPackets, gSerialBridgeRxBytes);
	UI_PrintStringSmallNormal(String, 2, 127, 4);

	sprintf(String, "E:%u EXIT=quit", gSerialBridgeRxErrors);
	UI_PrintStringSmallNormal(String, 2, 127, 5);

	ST7565_BlitFullScreen();
}

#endif
