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

#ifndef ESP8266SOCKET_H
#define ESP8266SOCKET_H

#define ESP8266_MAX_NUMBER_SOCKETS 	 		5
#define ESP8266_WAIT_MS				100

class ESP8266Socket{

public:
    ESP8266Socket(int fd);

    int connect(const char* host, const int port);

    bool is_connected(void);

    int close(bool shutdown=false);

    void set_blocking(bool blocking, unsigned int timeout);

    int send(char* data, int length);

    int send_all(char* data, int length);

    int receive(char* data, int length);

    int receive_all(char* data, int length);

private:
    bool _is_socket_connected;
    int m_fd;
    unsigned int m_timeout_ms;
};

#endif

