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

#include "ESP8266Socket.h"
#include "mbed.h"
#include "ESP8266InterfaceCWrapper.h"

ESP8266Socket::ESP8266Socket(int fd) :
	_is_socket_connected(false), m_timeout_ms(0){
	if (isEsp8266IfDown()){
		while (!turnEsp8266IfUp()); //block until if is up
	}
	if (!isEsp8266APconnected()){
		while (!connectToAP());  //block until is connected to AP
	}
	if (fd < ESP8266_MAX_NUMBER_SOCKETS){
		m_fd = fd;
	}
}

int ESP8266Socket::connect(const char* host, const int port) {
	wait_ms(ESP8266_WAIT_MS);
	if(esp8266connect("TCP",m_fd,host, port)){
		_is_socket_connected = true;
		return 0;
	}
    return -1;
}

bool ESP8266Socket::is_connected(void) {
    return _is_socket_connected;
}

int ESP8266Socket::close(bool shutdown){
	wait_ms(ESP8266_WAIT_MS);
	if (esp8266close(m_fd)){
		_is_socket_connected = false;
		if (shutdown){
			powerCycleESP8266(); // TODO:: last resort, not enabled
		}
		return 0;
	}
	_is_socket_connected = false;
	return -1;
}

void ESP8266Socket::set_blocking(bool blocking, unsigned int timeout) {
	//TODO:: handle blocking case, but not for http client
	if (!blocking){
		m_timeout_ms = timeout;
	}
}

int ESP8266Socket::send(char* data, int length) {
	Timer t;
	//TODO:: handle 2048+ bytes
    if ((m_fd < 0) || !_is_socket_connected || length > 2047)
        return -1;

    int time_to_wait = ESP8266_WAIT_MS;
    t.start();
    do{
		wait_ms(time_to_wait); //wait a bit, helps both in retry and consecutive sends...
		if (esp8266send(m_fd, data, (uint32_t) length)){
			//sent, ok
			t.stop();
			return length;
		}
		time_to_wait *=2; //double waiting time for next trial
		//retry
    }while(t.read_ms() <= m_timeout_ms);
    t.stop();
    //not sent
    return -1;
}

// -1 if unsuccessful, else number of bytes written
int ESP8266Socket::send_all(char* data, int length) {
	return send(data,length);
}

int ESP8266Socket::receive(char* data, int length) {
	Timer t;
    if ((m_fd < 0) || !_is_socket_connected)
        return -1;

    t.start();
    do{
		int n = esp8266recv(m_fd, data, (uint32_t) length);
		if (n>0){
			if (n > length){ // if packet > 1024 bytes
				//buffer overflow
				return -1;
			}
			//recv ok
			return n;
		}
		//retry
    }while(t.read_ms() <= m_timeout_ms);
    t.stop();
	//nothing received
    return 0;
}

// -1 if unsuccessful, else number of bytes received
int ESP8266Socket::receive_all(char* data, int length) {
	return receive(data, length);
}


