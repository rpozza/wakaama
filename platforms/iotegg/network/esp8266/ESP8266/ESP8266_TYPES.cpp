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

#include "ESP8266_TYPES.h"

WiFiStationAccessPoint::WiFiStationAccessPoint()
{
    memset(&_station_ap, 0, sizeof(_station_ap));
}

WiFiStationAccessPoint::WiFiStationAccessPoint(wifi_station_ap_t station_ap)
{
    _station_ap = station_ap;
}

const char *WiFiStationAccessPoint::get_ssid() const
{
    return _station_ap.ssid;
}

const uint8_t *WiFiStationAccessPoint::get_bssid() const
{
    return _station_ap.bssid;
}

wifi_security_t WiFiStationAccessPoint::get_security() const
{
    return _station_ap.security;
}

int8_t WiFiStationAccessPoint::get_rssi() const
{
    return _station_ap.rssi;
}

uint8_t WiFiStationAccessPoint::get_channel() const
{
    return _station_ap.channel;
}
