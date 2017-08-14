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

#ifndef ESP8266_CONFIG
#define ESP8266_CONFIG

//UART CONFIG PARAMETERS
#define UART_BUFFER_SIZE 											1024
#define DEFAULT_BAUD_RATE 											115200
#define FAST_BAUD_RATE 												921600

//// Number of allowed sockets
//#define ESP8266_SOCKET_COUNT 										5
// to move in sys/socket


// Various timeouts for different ESP8266 operations
#define ESP8266_CONNECT_TIMEOUT 									15000
#define ESP8266_SEND_TIMEOUT    									500
#define ESP8266_RECV_TIMEOUT    									0
#define ESP8266_MISC_TIMEOUT    									500

// Firmware version
#define ESP8266_VERSION 											2

// Pinout config
#define ESP8266_TX_PIN 												P4_28
#define ESP8266_RX_PIN												P4_29
#define ESP8266_RESET_PIN 											P0_4

// Debug
#define ESP8266_VERBOSE 											true

// ESP WiFi Parameters
#define ESP8266_MODE_STA											1
#define ESP8266_MODE_AP												2
#define ESP8266_MODE_STA_AND_AP										3

#define ESP8266_DHCP_ENABLE 										true
#define ESP8266_DHCP_SOFTAP 										0
#define ESP8266_DHCP_STATION 										1
#define ESP8266_DHCP_BOTH	 										2

// AP CONFIG
#ifndef LWM2M_WIFI_SSID
#define LWM2M_WIFI_SSID												"MGTestAP"
#endif

#ifndef LWM2M_WIFI_PSWD
#define LWM2M_WIFI_PSWD												"homesense"
#endif

#define ESP8266_UDP_PEER_NOCHANGE 									0
#define ESP8266_UDP_PEER_CHANGE_ONCE 								1
#define ESP8266_UDP_PEER_CHANGE_MULTIPLE 							2


#endif
