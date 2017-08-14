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

#ifndef ESP8266_INTERFACE_C_WRAPPER_H
#define ESP8266_INTERFACE_C_WRAPPER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool isEsp8266IfUp (void);
bool isEsp8266IfDown (void);

bool turnEsp8266IfUp(void);
bool turnEsp8266IfDown(void);

bool isEsp8266APconnected (void);
bool connectToAP(void);
bool disconnectFromAP(void);

const char *getIPAddress(void);
const char *getMACAddress(void);
bool setMACAddress(const char * macAddress);
const char *getGateway(void);
const char *getNetmask(void);

int getRSSI(void);
bool getHostByName(const char* name, char* ip);
int ping(const char *name);

uint32_t toU32_IPv4(const char* ipv4address);
uint16_t toU16_port(const char* port);

// Sockets handling
bool esp8266close(int fd);
bool esp8266bind(int fd, const char *localAddress, int localPort);
bool esp8266connect(int fd, const char *remoteAddress, int remotePort);
bool esp8266sendto(int fd, const void *data, uint32_t amount, const char *remoteip, int port);
bool powerCycleESP8266(void);
int32_t esp8266recvfrom(int fd, char *ipv4_addr, int *port, void *data, uint32_t amount);

#ifdef __cplusplus
}
#endif

#endif
