/* ESP8266 Example
 * Copyright (c) 2015 ARM Limited
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
 */

#include "ESP8266.h"
#include "ESP8266_CONFIG.h"
#include "mbed_debug.h"

ESP8266::ESP8266(PinName tx, PinName rx, PinName reset, bool debug)
    : _serial(tx, rx, UART_BUFFER_SIZE), _parser(_serial), _reset(reset,1), dbg_on(debug)
    , _packets(0), _packets_end(&_packets)
{
    _serial.baud(DEFAULT_BAUD_RATE);
    _parser.debugOn(debug);
}

int ESP8266::get_firmware_version()
{
    _parser.send("AT+GMR");
    int version;
    if(_parser.recv("SDK version:%d", &version) && _parser.recv("OK")) {
        return version;
    } else { 
        // Older firmware versions do not prefix the version with "SDK version: "
        return -1;
    }
    
}

bool ESP8266::test_ok(){
	return _parser.send("AT")
			&& _parser.recv("OK");
}

bool ESP8266::autobaudrate_init(){
	bool abr_ok = false;
    for (int i = 0; i < 2; i++) {
		if(test_ok()){ // default
			abr_ok = true;
			break;
		}
		_serial.baud(FAST_BAUD_RATE);
		if(test_ok()){ // fast
			abr_ok = true;
			break;
		}
		_serial.baud(DEFAULT_BAUD_RATE);
    }
    if (!abr_ok){
    	return false;
    }
    uart_configure(FAST_BAUD_RATE);
    _serial.baud(FAST_BAUD_RATE);
    return true;
}

bool ESP8266::startup(int mode)
{

    //only 3 valid modes
    if(mode < 1 || mode > 3) {
        return false;
    }

    bool success = _parser.send("AT+CWMODE_CUR=%d", mode)
        && _parser.recv("OK")
        && _parser.send("AT+CIPMUX=1")
        && _parser.recv("OK")
    	&& _parser.send("AT+CIPDINFO=1") // this require a change within packet handler to parse correctly
    	&& _parser.recv("OK");

    _parser.oob("+IPD", this, &ESP8266::_packet_handler);

    return success;
}

bool ESP8266::hw_reset(void)
{
    _reset.write(0); //GND
    wait_ms(100);
    _reset.write(1); //VDD

    debug_if(dbg_on, "Reset Power Cycle H-L-H\r\n");
    return true;
}

bool ESP8266::sw_reset(void)
{
    for (int i = 0; i < 2; i++) {
        if (_parser.send("AT+RST")
            && _parser.recv("OK\r\nready")) {
            return true;
        }
    }

    return false;
}

bool ESP8266::uart_configure(int baudrate){
	return _parser.send("AT+UART_CUR=%d,8,1,0,0", baudrate) //NB: keep default 8 bits, 1 stop bit, no parity, no flow control
	        && _parser.recv("OK");
}

bool ESP8266::dhcp(bool enabled, int mode)
{
    //only 3 valid modes
    if(mode < 0 || mode > 2) {
        return false;
    }

    return _parser.send("AT+CWDHCP_CUR=%d,%d", enabled?1:0, mode)
        && _parser.recv("OK");
}

bool ESP8266::connect(const char *ap, const char *passPhrase)
{
    return _parser.send("AT+CWJAP_CUR=\"%s\",\"%s\"", ap, passPhrase)
        && _parser.recv("OK");
}

bool ESP8266::disconnect(void)
{
    return _parser.send("AT+CWQAP") && _parser.recv("OK");
}

const char *ESP8266::getIPAddress(void)
{
    if (!(_parser.send("AT+CIFSR")
        && _parser.recv("+CIFSR:STAIP,\"%15[^\"]\"", _ip_buffer)
        && _parser.recv("OK"))) {
        return 0;
    }

    return _ip_buffer;
}

const char *ESP8266::getMACAddress(void)
{
    if (!(_parser.send("AT+CIFSR")
        && _parser.recv("+CIFSR:STAMAC,\"%17[^\"]\"", _mac_buffer)
        && _parser.recv("OK"))) {
        return 0;
    }

    return _mac_buffer;
}

bool ESP8266::setMACAddress(const char * mac_buffer)
{
    if (!(_parser.send("AT+CIPSTAMAC=\"%17[^\"]\"", mac_buffer)
        && _parser.recv("OK"))) {
        return true;
    }

    return false;
}

const char *ESP8266::getGateway()
{
    if (!(_parser.send("AT+CIPSTA_CUR?")
        && _parser.recv("+CIPSTA_CUR:gateway:\"%15[^\"]\"", _gateway_buffer)
        && _parser.recv("OK"))) {
        return 0;
    }

    return _gateway_buffer;
}

const char *ESP8266::getNetmask()
{
    if (!(_parser.send("AT+CIPSTA_CUR?")
        && _parser.recv("+CIPSTA_CUR:netmask:\"%15[^\"]\"", _netmask_buffer)
        && _parser.recv("OK"))) {
        return 0;
    }

    return _netmask_buffer;
}

int8_t ESP8266::getRSSI()
{
    int8_t rssi;
    char bssid[18];

   if (!(_parser.send("AT+CWJAP_CUR?")
        && _parser.recv("+CWJAP_CUR::\"%*[^\"]\",\"%17[^\"]\"", bssid)
        && _parser.recv("OK"))) {
        return 0;
    }

    if (!(_parser.send("AT+CWLAP=\"\",\"%s\",", bssid)
        && _parser.recv("+CWLAP:(%*d,\"%*[^\"]\",%hhd,", &rssi)
        && _parser.recv("OK"))) {
        return 0;
    }

    return rssi;
}

bool ESP8266::isConnected(void)
{
    return getIPAddress() != 0;
}

int ESP8266::scan(WiFiStationAccessPoint *res, unsigned limit)
{
    unsigned cnt = 0;
    wifi_station_ap_t ap;

    if (!_parser.send("AT+CWLAP")) {
        return WIFI_ERROR_SCAN;
    }

    while (recv_ap(&ap)) {
        if (cnt < limit) {
            res[cnt] = WiFiStationAccessPoint(ap);
        }

        cnt++;
        if (limit != 0 && cnt >= limit) {
            break;
        }
    }

    return cnt;
}

bool ESP8266::open(const char *type, int id, const char* addr, int port)
{
    //IDs only 0-4
    if(id > 4) {
        return false;
    }
    return _parser.send("AT+CIPSTART=%d,\"%s\",\"%s\",%d", id, type, addr, port)
        && _parser.recv("OK");
}

bool ESP8266::open_f2(const char *type, int id, const char* remoteaddr, int remoteport, int udplocalport, int udpmode)
{
    //IDs only 0-4
    if(id > 4) {
        return false;
    }
    return _parser.send("AT+CIPSTART=%d,\"%s\",\"%s\",%d,%d,%d", id, type, remoteaddr, remoteport, udplocalport, udpmode)
        && _parser.recv("OK");
}


bool ESP8266::dns_lookup(const char* name, char* ip)
{
    return _parser.send("AT+CIPDOMAIN=\"%s\"", name) && _parser.recv("+CIPDOMAIN:%s%*[\r]%*[\n]", ip);
}

int ESP8266::ping(const char *name){
	int rtt_ms;
	if (_parser.send("AT+PING=\"%s\"", name)
		&& _parser.recv("+%d",&rtt_ms)
        && _parser.recv("OK")) {
		return rtt_ms;
	}
	return -1;
}

bool ESP8266::send(int id, const void *data, uint32_t amount)
{
    //Just one try, even if device is busy, retry handled upper layers
	if (_parser.send("AT+CIPSEND=%d,%d", id, amount)
		&& _parser.recv(">")
		&& _parser.write((char*)data, (int)amount) >= 0) {
		return true;
	}
    return false;
}

bool ESP8266::sendto(int id, const void *data, uint32_t amount, const char *remoteip, int port)
{
    //May take a second try if device is busy
    for (unsigned i = 0; i < 2; i++) {
        if (_parser.send("AT+CIPSEND=%d,%d,\"%s\",%d", id, amount, remoteip, port)
            && _parser.recv(">")
            && _parser.write((char*)data, (int)amount) >= 0) {
            return true;
        }
    }

    return false;
}

void ESP8266::_packet_handler()
{
    int id;
    uint32_t amount;
    char ip_addr[16];
    int port;

    // parse out the packet
    if (!_parser.recv(",%d,%d,%15[^,],%d:", &id, &amount, ip_addr, &port)) {
        return;
    }

    struct packet *packet = (struct packet*)malloc(
            sizeof(struct packet) + amount);
    if (!packet) {
        return;
    }

    packet->id = id;
    packet->len = amount;
    memcpy(packet->ipv4_rem, ip_addr, 16);
    packet->port_rem = port;
    packet->next = 0;

    if (!(_parser.read((char*)(packet + 1), amount))) {
        free(packet);
        return;
    }

    // append to packet list
    *_packets_end = packet;
    _packets_end = &packet->next;
}

int32_t ESP8266::recvfrom(int id, char *ipv4_addr, int *port, void *data, uint32_t amount)
{
    while (true) {
        // check if any packets are ready for us
        for (struct packet **p = &_packets; *p; p = &(*p)->next) {
            if ((*p)->id == id) {
                struct packet *q = *p;

                if (q->len <= amount) { // Return and remove full packet
                    memcpy(ipv4_addr, q->ipv4_rem, 16);
                    (*port) = q->port_rem;

                	memcpy(data, q+1, q->len);

                    if (_packets_end == &(*p)->next) {
                        _packets_end = p;
                    }
                    *p = (*p)->next;

                    uint32_t len = q->len;
                    free(q);
                    return len;
                } else { // return only partial packet
                    memcpy(ipv4_addr, q->ipv4_rem, 16);
                    (*port) = q->port_rem;

                    memcpy(data, q+1, amount);

                    q->len -= amount;
                    memmove(q+1, (uint8_t*)(q+1) + amount, q->len);

                    return amount;
                }
            }
        }

        // Wait for inbound packet
        if (!_parser.recv("OK")) {
            return -1;
        }
    }
}

bool ESP8266::close(int id)
{
    //May take a second try if device is busy
    for (unsigned i = 0; i < 2; i++) {
        if (_parser.send("AT+CIPCLOSE=%d", id)
            && _parser.recv("OK")) {
            return true;
        }
    }

    return false;
}

void ESP8266::setTimeout(uint32_t timeout_ms)
{
    _parser.setTimeout(timeout_ms);
}

bool ESP8266::readable()
{
    return _serial.readable();
}

bool ESP8266::writeable()
{
    return _serial.writeable();
}

void ESP8266::attach(void (*func) (void)){
    _serial.attach((modserialfunc_t)func);
}

bool ESP8266::recv_ap(wifi_station_ap_t *ap)
{
    int sec;
    bool ret = _parser.recv("+CWLAP:(%d,\"%32[^\"]\",%hhd,\"%hhx:%hhx:%hhx:%hhx:%hhx:%hhx\",%d", &sec, ap->ssid,
                            &ap->rssi, &ap->bssid[0], &ap->bssid[1], &ap->bssid[2], &ap->bssid[3], &ap->bssid[4],
                            &ap->bssid[5], &ap->channel);

    ap->security = sec < 5 ? (wifi_security_t)sec : UNKNOWN;

    return ret;
}
