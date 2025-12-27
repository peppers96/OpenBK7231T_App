// Basic thermostat driver - links a temperature sensor to a relay channel
#ifndef _DRV_THERMOSTAT_H_
#define _DRV_THERMOSTAT_H_

#include "../httpserver/new_http.h"

void Thermostat_driver_Init();
void Thermostat_OnEverySecond();
void Thermostat_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState);

#if ENABLE_HA_DISCOVERY
struct HassDeviceInfo_s;
typedef struct HassDeviceInfo_s HassDeviceInfo;
HassDeviceInfo* Thermostat_GetHASSInfo();
#endif

#endif
