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

#include "ESP8266.h"
#include "ESP8266_CONFIG.h"
#include "ESP8266InterfaceCWrapper.h"
#include "mbed_debug.h"
#include "mbed_api_wrapper.h"

static ESP8266 * esp8266if = NULL;
static bool esp8266APconnected = false;

bool isEsp8266IfUp (void){
	if (esp8266if != NULL){
		return true;
	}
	return false;
}

bool isEsp8266IfDown (void){
	if (esp8266if == NULL){
		return true;
	}
	return false;
}

bool turnEsp8266IfUp(void){
	if (isEsp8266IfDown()){
		esp8266if = new ESP8266(ESP8266_TX_PIN, ESP8266_RX_PIN, ESP8266_RESET_PIN, ESP8266_VERBOSE);
		if (isEsp8266IfUp()){
			esp8266if->hw_reset(); //hw reboot interface
			wait_ms(ESP8266_MISC_TIMEOUT);

			esp8266if->setTimeout(ESP8266_MISC_TIMEOUT);
			esp8266if->sw_reset(); //sw reboot to be on the safe side
			if (esp8266if->autobaudrate_init()){
				debug_if(ESP8266_VERBOSE,"PHY> IF UP!\r\n");
				return true; //setup fast uart speed
			}
		}
		debug_if(ESP8266_VERBOSE,"ERROR> PHY IF setup incomplete\r\n");
		return false;
	}
	debug_if(ESP8266_VERBOSE,"ERROR> PHY IF already UP\r\n");
}

bool turnEsp8266IfDown(void){
	if (isEsp8266IfUp()){
		delete esp8266if;
		esp8266if = NULL;
		if (isEsp8266IfDown()){
			debug_if(ESP8266_VERBOSE,"PHY> IF DOWN!\r\n");
			return true;
		}
		return false;
	}
	debug_if(ESP8266_VERBOSE,"ERROR> PHY IF already DOWN\r\n");
}

bool isEsp8266APconnected (void){
	return esp8266APconnected;
}

bool connectToAP(void){
	esp8266if->setTimeout(ESP8266_MISC_TIMEOUT);
	if(esp8266if->get_firmware_version() != ESP8266_VERSION){
		debug_if(ESP8266_VERBOSE,"PHY> WRONG ESP_FW VERSION\r\n");
		return false;
	}
	esp8266if->setTimeout(ESP8266_CONNECT_TIMEOUT);
	if (!esp8266if->startup(ESP8266_MODE_STA)){
		debug_if(ESP8266_VERBOSE,"PHY> AP MODE CONFIG FAILED\r\n");
		return false;
	}
	if(!esp8266if->dhcp(ESP8266_DHCP_ENABLE,ESP8266_DHCP_STATION)){
		debug_if(ESP8266_VERBOSE,"PHY> DHCP CONFIG FAILED\r\n");
		return false;
	}
	if(!esp8266if->connect(LWM2M_WIFI_SSID,LWM2M_WIFI_PSWD)){
		debug_if(ESP8266_VERBOSE,"PHY> FAILED CONNECT TO SSID:PSWD\r\n");
		return false;
	}
	if(!esp8266if->getIPAddress()){
		debug_if(ESP8266_VERBOSE,"PHY> IP ASSIGNMENT FAILURE\r\n");
		return false;
	}
	esp8266APconnected = true;
	return true;
}

bool disconnectFromAP(void){
	esp8266if->setTimeout(ESP8266_MISC_TIMEOUT);
	if (!esp8266if->disconnect()){
		debug_if(ESP8266_VERBOSE,"PHY> DISCONNECTION ERROR\r\n");
		return false;
	}
	esp8266APconnected = false;
	return true;
}

const char *getIPAddress(void){
	esp8266if->setTimeout(ESP8266_MISC_TIMEOUT);
	if (isEsp8266IfUp()){
		return esp8266if->getIPAddress();
	}
	return 0;
}

const char *getMACAddress(void){
	esp8266if->setTimeout(ESP8266_MISC_TIMEOUT);
	if (isEsp8266IfUp()){
		return esp8266if->getMACAddress();
	}
	return 0;
}

bool setMACAddress(const char * macAddress){
	esp8266if->setTimeout(ESP8266_MISC_TIMEOUT);
	if (isEsp8266IfUp()){
		return esp8266if->setMACAddress(macAddress);
	}
	return false;
}

const char *getGateway(void){
	esp8266if->setTimeout(ESP8266_MISC_TIMEOUT);
	if (isEsp8266IfUp()){
		return esp8266if->getGateway();
	}
	return 0;
}

const char *getNetmask(void){
	esp8266if->setTimeout(ESP8266_MISC_TIMEOUT);
	if (isEsp8266IfUp()){
		return esp8266if->getNetmask();
	}
	return 0;
}

int getRSSI(void){
	esp8266if->setTimeout(ESP8266_MISC_TIMEOUT);
	if (isEsp8266IfUp()){
		return esp8266if->getRSSI();
	}
	return 0;
}

bool getHostByName(const char* name, char* ip){
	esp8266if->setTimeout(ESP8266_MISC_TIMEOUT);
	if (isEsp8266IfUp()){
		return esp8266if->dns_lookup(name, ip);
	}
	return 0;
}

int ping(const char *name){
	esp8266if->setTimeout(ESP8266_MISC_TIMEOUT);
	if (isEsp8266IfUp()){
		return esp8266if->ping(name);
	}
	return -1;
}

uint32_t toU32_IPv4(const char* ipv4address){
	unsigned int byte3;
	unsigned int byte2;
	unsigned int byte1;
	unsigned int byte0;
	uint32_t retval = -1;
	sscanf(ipv4address, "%u.%u.%u.%u", &byte3, &byte2, &byte1, &byte0);
	if ((byte3 < 256) && (byte2 < 256) && (byte1 < 256) && (byte0 < 256)){
		retval = (byte3 << 24) + (byte2 << 16) + (byte1 << 8) +  byte0;
    }
	return retval;
}

uint16_t toU16_port(const char* port){
	int portvalue = -1;
	uint16_t retval = -1;
	sscanf(port, "%d", &portvalue);
	if ((portvalue >=0) && (portvalue <= 65536)){
		retval = (uint16_t) portvalue;
	}
	return retval;
}

bool esp8266close(int fd){
	esp8266if->setTimeout(ESP8266_MISC_TIMEOUT);
	return esp8266if->close(fd);
}

bool esp8266bind(int fd, const char *localAddress, int localPort){
	esp8266if->setTimeout(ESP8266_MISC_TIMEOUT);
	return esp8266if->open_f2("UDP", fd, "0.0.0.0", localPort, localPort, ESP8266_UDP_PEER_CHANGE_MULTIPLE);
}

bool esp8266connect(int fd, const char *remoteAddress, int remotePort){
	esp8266if->setTimeout(ESP8266_MISC_TIMEOUT);
	return esp8266if->open("UDP", fd, remoteAddress, remotePort);
}

bool esp8266sendto(int fd, const void *data, uint32_t amount, const char *remoteip, int port){
	esp8266if->setTimeout(ESP8266_SEND_TIMEOUT);
	return esp8266if->sendto(fd, data, amount, remoteip, port);
}

bool powerCycleESP8266(void){
	if (turnEsp8266IfDown()){
		turnEsp8266IfUp(); //again with a reboot :-)
		esp8266APconnected = false; // this is obvious because we rebooted esp
		while (!connectToAP());
		// try to setup again wireless comms
		return true;
	}
	return false;
}

int32_t esp8266recvfrom(int fd, char *ipv4_addr, int *port, void *data, uint32_t amount){
	esp8266if->setTimeout(ESP8266_RECV_TIMEOUT);
	return esp8266if->recvfrom(fd, ipv4_addr, port, data, amount);
}

void dbgOnParserON(bool on){
	esp8266if->debugOnParser(on);
}
