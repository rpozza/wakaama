/*
 * Copyright [2016] [Riccardo Pozza]
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

#ifndef ESP8266_WRAPPER_H_
#define ESP8266_WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

void connectESP8266toAP();
void rebootESP8266();

int close_connection(int fd);
int bind_connection(int fd,uint32_t local_address, uint16_t local_port);
int connect_connection(int fd,uint32_t remote_address, uint16_t remote_port);
int sendto_connection(char * buf, size_t length, uint32_t remote_address, uint16_t remote_port, int fd);
int recvfrom_connection(char * buf, size_t length, uint32_t * remote_address, uint16_t * remote_port, int fd);

int get_rssi_indication(void);
char * get_local_ip(char* buffer);
uint32_t get_local_ipv4();
uint32_t convert_ipv4_to_number(const char* buffer);
uint16_t convert_port_to_number(const char* buffer);
char * get_local_mac(char* buffer);
char * get_router_bssid(char* buffer);
char * get_router_ssid(char* buffer);

void get_connection_status(void);
int ping_server(const char * hostname);

#ifdef __cplusplus
}
#endif

#endif
