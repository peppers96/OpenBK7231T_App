#include "drv_thermostat.h"
#include "../new_common.h"
#include "../new_cfg.h"
#include "../logging/logging.h"
#include "../new_pins.h"
#include "../cmnds/cmd_public.h"
#include "../mqtt/new_mqtt.h"
#if ENABLE_HA_DISCOVERY
#include "../httpserver/hass.h"
#endif
#if ENABLE_DRIVER_DS1820_FULL
#include "drv_ds1820_full.h"
#endif
#if ENABLE_DRIVER_DS1820
#include "drv_ds1820_simple.h"
#endif

static int g_relayChannel = -1; // channel index for relay (if -1, will pick first relay)
static int g_tempChannel = -1; // channel index for specific temperature channel, or -1 to auto
static float g_setpoint = 20.0f;
static float g_hysteresis = 0.5f; // degrees C
static int g_setpointChannel = -1; // if >=0, read setpoint value from this channel (float)
static int g_isActive = 1;
static int g_isActiveChannel = -1; // if >=0, read active state from this channel (digital: 0 off, !=0 on)
static int g_lastRelayState = -1; // -1 unknown
static float g_alarmThreshold = 80.0f; // degrees C - if exceeded, force relay off
static int g_staleTimeoutSeconds = 60; // seconds - if no recent DS18B20 reading, force relay off
static int g_driverEnabled = 0; // flag: driver is active, manual relay commands are blocked
static int g_initialized = 0; // flag: driver has been initialized via startDriver

// MQTT publication tracking - publish only on change
static float g_lastPublishedTemp = -999.0f;
static float g_lastPublishedSetpoint = -999.0f;
static int g_lastPublishedMode = -1;

// Command handler for MQTT setpoint control: THERMOSTAT_Set <temperature>
static commandResult_t Cmd_Thermostat_Set(const void* context, const char* cmd, const char* args, int flags) {
    int rc = Tokenizer_GetArgsCount();
    
    // If only one argument, treat it as setpoint value from Home Assistant
    if (rc == 1) {
        g_setpoint = Tokenizer_GetArgFloatDefault(1, g_setpoint);
        addLogAdv(LOG_INFO, LOG_FEATURE_GENERAL, "Thermostat setpoint changed to %.2f°C\n", g_setpoint);
        // Force republish on next cycle
        g_lastPublishedSetpoint = -999.0f;
        return CMD_RES_OK;
    }
    
    // If multiple arguments, treat it as configuration (from startDriver or manual config)
    if (rc >= 1) {
        g_relayChannel = Tokenizer_GetArgIntegerDefault(1, -1);
    }
    if (rc >= 2) {
        g_setpoint = Tokenizer_GetArgFloatDefault(2, g_setpoint);
    }
    if (rc >= 3) {
        g_hysteresis = Tokenizer_GetArgFloatDefault(3, g_hysteresis);
    }
    if (rc >= 4) {
        g_tempChannel = Tokenizer_GetArgIntegerDefault(4, -1);
    }
    if (rc >= 5) {
        g_setpointChannel = Tokenizer_GetArgIntegerDefault(5, -1);
    }
    if (rc >= 6) {
        g_isActiveChannel = Tokenizer_GetArgIntegerDefault(6, -1);
    }
    if (rc >= 7) {
        g_alarmThreshold = Tokenizer_GetArgFloatDefault(7, g_alarmThreshold);
    }
    if (rc >= 8) {
        g_staleTimeoutSeconds = Tokenizer_GetArgIntegerDefault(8, g_staleTimeoutSeconds);
    }
    addLogAdv(LOG_INFO, LOG_FEATURE_GENERAL, "Thermostat configured: relayCH=%i setpoint=%.2fC setpointCH=%i hyst=%.2f tempCH=%i isActiveCH=%i alarm=%.2f\n", g_relayChannel, g_setpoint, g_setpointChannel, g_hysteresis, g_tempChannel, g_isActiveChannel, g_alarmThreshold);
    return CMD_RES_OK;
}


static commandResult_t Cmd_Thermostat_Enable(const void* context, const char* cmd, const char* args, int flags) {
    int rc = Tokenizer_GetArgsCount();
    if (rc >= 1) {
        g_isActive = Tokenizer_GetArgIntegerDefault(1, g_isActive);
        addLogAdv(LOG_INFO, LOG_FEATURE_GENERAL, "Thermostat %s\n", g_isActive ? "enabled" : "disabled");
    }
    return CMD_RES_OK;
}

void Thermostat_driver_Init() {
    // Usage: startDriver THERMOSTAT [relayChannel=-1] [setpoint=20.0] [hysteresis=0.5] [tempChannel=-1]
    g_relayChannel = Tokenizer_GetArgIntegerDefault(1, -1);
    g_setpoint = Tokenizer_GetArgFloatDefault(2, g_setpoint);
    g_hysteresis = Tokenizer_GetArgFloatDefault(3, g_hysteresis);
    g_tempChannel = Tokenizer_GetArgIntegerDefault(4, -1);
    g_setpointChannel = Tokenizer_GetArgIntegerDefault(5, -1);
    g_isActiveChannel = Tokenizer_GetArgIntegerDefault(6, -1);
    g_alarmThreshold = Tokenizer_GetArgFloatDefault(7, g_alarmThreshold);
    g_staleTimeoutSeconds = Tokenizer_GetArgIntegerDefault(8, g_staleTimeoutSeconds);

    CMD_RegisterCommand("THERMOSTAT_Set", Cmd_Thermostat_Set, NULL);
    // allow toggling on/off via thermostat control; manual relay commands ignored by OnEverySecond
    CMD_RegisterCommand("THERMOSTAT_Enable", Cmd_Thermostat_Enable, NULL);

    g_driverEnabled = 1; // mark driver as active
    g_initialized = 1; // mark driver as initialized
    addLogAdv(LOG_INFO, LOG_FEATURE_GENERAL, "Thermostat driver started: relayCH=%i setpoint=%.2fC hyst=%.2f tempCH=%i alarm=%.2f\n", g_relayChannel, g_setpoint, g_hysteresis, g_tempChannel, g_alarmThreshold);
}

void Thermostat_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState) {
    float curTemp = 0.0f;
    bool haveTemp = false;
    if (g_tempChannel >= 0) {
        curTemp = CHANNEL_GetFinalValue(g_tempChannel);
        haveTemp = true;
    } else {
        haveTemp = CHANNEL_GetGenericTemperature(&curTemp);
    }
    float displayedSetpoint = g_setpoint;
    if (g_setpointChannel >= 0) {
        displayedSetpoint = CHANNEL_GetFinalValue(g_setpointChannel);
    }

    float displayedAlarm = g_alarmThreshold;
    int displayedStale = g_staleTimeoutSeconds;
    // if DS18 driver available, report sensor presence for the configured temp channel
#if ENABLE_DRIVER_DS1820_FULL
    int ds_age = -2;
    if (g_tempChannel >= 0) ds_age = DS18B20_get_last_read_seconds_for_channel(g_tempChannel);
#elif ENABLE_DRIVER_DS1820
    int ds_age = -2;
    if (g_tempChannel >= 0) ds_age = DS1820_get_last_read_seconds_for_channel(g_tempChannel);
#else
    int ds_age = -2;
#endif

    hprintf255(request, "<h5>Thermostat: setpoint=%.2f C hysteresis=%.2f C</h5>", displayedSetpoint, g_hysteresis);
    if (haveTemp) {
        hprintf255(request, "<p>Current: %.2f C</p>", curTemp);
    } else {
        hprintf255(request, "<p>Current: N/A</p>");
    }
    hprintf255(request, "<p>Relay channel: %i</p>", g_relayChannel);
    hprintf255(request, "<p>Setpoint source: %s %i</p>", (g_setpointChannel >= 0) ? "channel" : "value", (g_setpointChannel >= 0) ? g_setpointChannel : (int)displayedSetpoint);
    hprintf255(request, "<p>Alarm threshold: %.2f C</p>", displayedAlarm);
    hprintf255(request, "<p>Stale timeout: %i s</p>", displayedStale);
    if (g_tempChannel >= 0 && ds_age != -2) {
        if (ds_age < 0) hprintf255(request, "<p>Sensor: <b>missing</b></p>");
        else hprintf255(request, "<p>Sensor: OK (last read %i s ago)</p>", ds_age);
    }
    if (g_isActiveChannel >= 0) {
        int active = CHANNEL_Get(g_isActiveChannel);
        hprintf255(request, "<p>Active (from channel %i): %s</p>", g_isActiveChannel, active ? "ON" : "OFF");
    } else {
        hprintf255(request, "<p>Active: %s</p>", g_isActive ? "ON" : "OFF");
    }
}

void Thermostat_OnEverySecond() {
    // Skip all logic until driver is properly initialized
    if (!g_initialized) {
        return;
    }
    
    // When thermostat is active, force relay OFF immediately if g_isActive is false
    // This overrides any manual relay commands; only thermostat logic can control relay
    if (g_driverEnabled && g_relayChannel >= 0) {
        int effectiveActive = g_isActive;
        if (g_isActiveChannel >= 0) {
            effectiveActive = CHANNEL_Get(g_isActiveChannel);
        }
        if (!effectiveActive && CHANNEL_Get(g_relayChannel) != 0) {
            CHANNEL_Set(g_relayChannel, 0, 0);
            addLogAdv(LOG_INFO, LOG_FEATURE_GENERAL, "Thermostat: g_isActive=0, forcing relay %i -> 0\n", g_relayChannel);
            g_lastRelayState = 0;
            return;
        }
    }

    // Always check temperature and alarm threshold even if thermostat is disabled
    float curTemp = 0.0f;
    bool haveTemp = false;
    if (g_tempChannel >= 0) {
        curTemp = CHANNEL_GetFinalValue(g_tempChannel);
        haveTemp = true;
    } else {
        haveTemp = CHANNEL_GetGenericTemperature(&curTemp);
    }
    // if using a specific DS18 temp channel, ensure sensor is present (and was read at least once)
#if ENABLE_DRIVER_DS1820_FULL
    if (g_tempChannel >= 0) {
        int age = DS18B20_get_last_read_seconds_for_channel(g_tempChannel);
        if (age < 0) {
            // sensor not present or never read
            if (g_relayChannel >= 0 && CHANNEL_Get(g_relayChannel) != 0) {
                CHANNEL_Set(g_relayChannel, 0, 0);
                addLogAdv(LOG_INFO, LOG_FEATURE_GENERAL, "Thermostat: sensor for channel %i not present or not read, forcing relay %i -> 0\n", g_tempChannel, g_relayChannel);
                g_lastRelayState = 0;
            }
            return;
        }
    }
#elif ENABLE_DRIVER_DS1820
    if (g_tempChannel >= 0) {
        int age = DS1820_get_last_read_seconds_for_channel(g_tempChannel);
        if (age < 0) {
            if (g_relayChannel >= 0 && CHANNEL_Get(g_relayChannel) != 0) {
                CHANNEL_Set(g_relayChannel, 0, 0);
                addLogAdv(LOG_INFO, LOG_FEATURE_GENERAL, "Thermostat: sensor for channel %i not present or not read, forcing relay %i -> 0\n", g_tempChannel, g_relayChannel);
                g_lastRelayState = 0;
            }
            return;
        }
    }
#endif
    if (!haveTemp) {
        // nothing to do: ensure relay is off for safety
        if (g_relayChannel >= 0 && CHANNEL_Get(g_relayChannel) != 0) {
            CHANNEL_Set(g_relayChannel, 0, 0);
            addLogAdv(LOG_INFO, LOG_FEATURE_GENERAL, "Thermostat: no temperature available, forcing relay %i -> 0\n", g_relayChannel);
            g_lastRelayState = 0;
        }
        return;
    }

    // determine relay channel if not configured (needed for alarm enforcement)
    if (g_relayChannel < 0) {
        for (int i = 0; i < CHANNEL_MAX; ++i) {
            if (CHANNEL_GetType(i) == ChType_Toggle) {
                g_relayChannel = i;
                break;
            }
        }
    }
    if (g_relayChannel < 0) return; // no relay available

    int curRelay = CHANNEL_Get(g_relayChannel);

    // check alarm threshold and enforce it
    float effectiveAlarm = g_alarmThreshold;
    
    // check stale DS18B20 reading if we are using a specific temp channel
    if (g_tempChannel >= 0 && g_staleTimeoutSeconds > 0) {
#if ENABLE_DRIVER_DS1820_FULL
        int age = DS18B20_get_last_read_seconds_for_channel(g_tempChannel);
#elif ENABLE_DRIVER_DS1820
        int age = DS1820_get_last_read_seconds_for_channel(g_tempChannel);
#else
        int age = -1;
#endif
        if (age >= 0 && age >= g_staleTimeoutSeconds) {
            if (curRelay != 0) {
                CHANNEL_Set(g_relayChannel, 0, 0);
                addLogAdv(LOG_INFO, LOG_FEATURE_GENERAL, "Thermostat STALE: temp age %i s >= %i s, forcing relay %i -> 0\n", age, g_staleTimeoutSeconds, g_relayChannel);
                g_lastRelayState = 0;
            }
            return; // stale reading - skip control
        }
    }
    if (curTemp >= effectiveAlarm) {
        // force relay off to prevent overheating
        if (curRelay != 0) {
            CHANNEL_Set(g_relayChannel, 0, 0);
            addLogAdv(LOG_INFO, LOG_FEATURE_GENERAL, "Thermostat ALARM: temp %.2f >= %.2f, forcing relay %i -> 0\n", curTemp, effectiveAlarm, g_relayChannel);
            g_lastRelayState = 0;
        }
        return; // alarm handled, skip normal control
    }

    // determine active: channel overrides static flag
    if (g_isActiveChannel >= 0) {
        int chv = CHANNEL_Get(g_isActiveChannel);
        if (!chv) return;
    } else {
        if (!g_isActive) return;
    }

    // effective setpoint (can be static float or read from a channel)
    float effectiveSetpoint = g_setpoint;
    if (g_setpointChannel >= 0) {
        effectiveSetpoint = CHANNEL_GetFinalValue(g_setpointChannel);
    }

    float halfH = g_hysteresis * 0.5f;
    int newRelay = curRelay;

    if (curRelay == 0) {
        if (curTemp < (effectiveSetpoint - halfH)) newRelay = 1;
    } else {
        if (curTemp > (effectiveSetpoint + halfH)) newRelay = 0;
    }

    if (newRelay != curRelay) {
        CHANNEL_Set(g_relayChannel, newRelay, 0);
        addLogAdv(LOG_INFO, LOG_FEATURE_GENERAL, "Thermostat changed relay %i -> %i (temp=%.2f set=%.2f hyst=%.2f)\n", g_relayChannel, newRelay, curTemp, effectiveSetpoint, g_hysteresis);
    }

    g_lastRelayState = newRelay;
    
    // Publish MQTT state topics for Home Assistant only on value changes
    #if ENABLE_MQTT
    {
        // Publish current temperature as channel value
        if (curTemp < g_lastPublishedTemp - 0.1f || curTemp > g_lastPublishedTemp + 0.1f) {
            char pubBuf[32];
            sprintf(pubBuf, "%.2f", curTemp);
            MQTT_QueuePublish(CFG_GetMQTTClientId(), "temp", pubBuf, 0);
            g_lastPublishedTemp = curTemp;
        }
        
        // Publish setpoint as channel value
        if (effectiveSetpoint < g_lastPublishedSetpoint - 0.1f || effectiveSetpoint > g_lastPublishedSetpoint + 0.1f) {
            char pubBuf[32];
            sprintf(pubBuf, "%.2f", effectiveSetpoint);
            MQTT_QueuePublish(CFG_GetMQTTClientId(), "setpoint", pubBuf, 0);
            g_lastPublishedSetpoint = effectiveSetpoint;
        }
        
        // Publish mode state only if changed
        int effectiveActive = g_isActive;
        if (g_isActiveChannel >= 0) {
            effectiveActive = CHANNEL_Get(g_isActiveChannel);
        }
        int currentMode = effectiveActive ? 1 : 0;
        if (currentMode != g_lastPublishedMode) {
            MQTT_QueuePublish(CFG_GetMQTTClientId(), "mode", effectiveActive ? "heat" : "off", 0);
            g_lastPublishedMode = currentMode;
        }
    }
    #endif
}

#if ENABLE_HA_DISCOVERY
HassDeviceInfo* Thermostat_GetHASSInfo() {
    // Use the built-in HASS HVAC helper function
    // Parameters: min_temp, max_temp, step, fanOptions, numFanOptions, swingOptions, numSwingOptions, swingHOptions, numSwingHOptions
    HassDeviceInfo* info = hass_createHVAC(5.0f, 40.0f, 0.5f, NULL, 0, NULL, 0, NULL, 0);
    
    if (info == NULL) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_GENERAL, "Thermostat: Failed to create HASS HVAC info");
        return NULL;
    }
    
    // Update name
    cJSON_ReplaceItemInObject(info->root, "name", cJSON_CreateString("Thermostat"));
    
    // Update temperature topics to use device channels
    // current_temperature_topic: receives numeric temperature value from channel
    cJSON_ReplaceItemInObject(info->root, "current_temperature_topic", cJSON_CreateString("~/temp"));
    
    // temperature_state_topic: receives current setpoint as numeric value from channel
    cJSON_ReplaceItemInObject(info->root, "temperature_state_topic", cJSON_CreateString("~/setpoint"));
    
    // Update command topic for setpoint (Home Assistant will send numeric value here)
    char cmdBuf[256];
    sprintf(cmdBuf, "cmnd/%s/THERMOSTAT_Set", CFG_GetMQTTClientId());
    cJSON_ReplaceItemInObject(info->root, "temperature_command_topic", cJSON_CreateString(cmdBuf));
    
    // Update mode topics to use device channels
    // mode_state_topic: receives "off" or "heat" as string
    cJSON_ReplaceItemInObject(info->root, "mode_state_topic", cJSON_CreateString("~/mode"));
    
    // mode_command_topic: receives "off" or "heat" and we convert to 0/1
    sprintf(cmdBuf, "cmnd/%s/THERMOSTAT_Enable", CFG_GetMQTTClientId());
    cJSON_ReplaceItemInObject(info->root, "mode_command_topic", cJSON_CreateString(cmdBuf));
    
    // Note: We use simple numeric values for temperature and 0/1 for mode enable/disable
    // The device will publish temp/setpoint as numbers and mode as "off"/"heat" strings
    
    return info;
}
#endif
