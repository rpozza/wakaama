/*
 * Copyright [2017] [Riccardo Pozza]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author:
 * Riccardo Pozza <r.pozza@surrey.ac.uk>
 */

#ifndef ESP8266_TYPES
#define ESP8266_TYPES

#include "MODSERIAL.h"

typedef void (*modserialfunc_t) (MODSERIAL_IRQ_INFO *);

enum wifi_error {
    WIFI_ERROR_OK                  =  0,
	WIFI_ERROR_SCAN				   = -1000
};

typedef enum wifi_security_t {
	NONE 		= 0x0,
	WEP 		= 0x1,
	WPA 		= 0x2,
	WPA2 		= 0x3,
	WPA_WPA2 	= 0x4,
	PAP			= 0x5,
	CHAP		= 0x6,
	UNKNOWN 	= 0xFF
} wifi_security_t;

typedef struct wifi_station_ap {
    char ssid[33]; /* 32 is what 802.11 defines as longest possible name; +1 for null termination */
    uint8_t bssid[6];
    wifi_security_t security;
    int8_t rssi;
    uint8_t channel;
} wifi_station_ap_t;

/** WiFiStationAccessPoint class.
    Accessory data model for WiFi Station AP storage
 */
class WiFiStationAccessPoint
{
public:
    WiFiStationAccessPoint();
    WiFiStationAccessPoint(wifi_station_ap_t station_ap);

    const char *get_ssid() const;

    const uint8_t *get_bssid() const;

    wifi_security_t get_security() const;

    int8_t get_rssi() const;

    uint8_t get_channel() const;

private:
    wifi_station_ap_t _station_ap;
};

#endif
