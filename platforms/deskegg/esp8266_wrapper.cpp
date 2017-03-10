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

#include "Esp8266WiFiTCPUDP.h"
#include "esp8266_wrapper.h"

static Esp8266WiFiTCPUDP * driverHandler = NULL;
static StationAccessPoint mobileGateway(LWM2M_WIFI_SSID, LWM2M_WIFI_PSWD,"");

void connectESP8266toAP(){
	if (driverHandler == NULL){

		Esp8266WiFiTCPUDP *wifi = new Esp8266WiFiTCPUDP();
		/* station mode */
		wifi->SetWiFiMode(WIFI_MODE_STA);
		/* enable dhcp */
		wifi->DHCPEnable(WIFI_DHCP_STATION, WIFI_DHCP_ENABLE);

		/* connect to gateway */
		bool is_connected = false;
		do {
			is_connected = wifi->ConnectToStationAP(mobileGateway);
		}while(!is_connected);

		/* multiple connections */
		wifi->EnableMultipleConnections();
		/* show remote info */
		wifi->EnableShowRemoteInReceive();

		// updates the new handler !! do not call more than one
		driverHandler = wifi;
	}
}

void rebootESP8266(){
	if (driverHandler != NULL){
		// hard reset
		// clean up old class
		delete driverHandler;
		// start a new one
		driverHandler = new Esp8266WiFiTCPUDP();

		driverHandler->SetWiFiMode(WIFI_MODE_STA);
		driverHandler->DHCPEnable(WIFI_DHCP_STATION, WIFI_DHCP_ENABLE);

		bool is_connected = false;
		do {
			is_connected = driverHandler->ConnectToStationAP(mobileGateway);
		}while(!is_connected);

		driverHandler->EnableMultipleConnections();
		driverHandler->EnableShowRemoteInReceive();
	}
}

int close_connection(int fd){
	if (driverHandler != NULL){
		if (driverHandler->ConnectionClose(fd)){
			return 1;
		}
	}
	return 0;
}

int bind_connection(int fd,uint32_t local_address, uint16_t local_port){
	string remoteAddress = "";
	char endpoint[20];
	if (driverHandler != NULL){
		uint8_t byte0 = local_address & 0xFF;
		uint8_t byte1 = (local_address >> 8) & 0xFF;
		uint8_t byte2 = (local_address >> 16) & 0xFF;
		uint8_t byte3 = (local_address >> 24) & 0xFF;
		snprintf(endpoint,20,"%d.%d.%d.%d",byte3, byte2, byte1, byte0);
		remoteAddress = string(endpoint); // local host
		driverHandler->ConnectionClose((uint8_t) fd); // precaution, TODO:: better handling
		if (driverHandler->UDPClientServerBind(remoteAddress, (uint32_t) local_port, (uint32_t) local_port, UDP_PEER_CHANGE_MULTIPLE, (uint8_t) fd)){
			return 1;
		}
	}
	return 0;
}

int connect_connection(int fd,uint32_t remote_address, uint16_t remote_port){
	string remoteAddress = "";
	char endpoint[20];
	if (driverHandler != NULL){
		uint8_t byte0 = remote_address & 0xFF;
		uint8_t byte1 = (remote_address >> 8) & 0xFF;
		uint8_t byte2 = (remote_address >> 16) & 0xFF;
		uint8_t byte3 = (remote_address >> 24) & 0xFF;
		snprintf(endpoint,20,"%d.%d.%d.%d",byte3, byte2, byte1, byte0);
		remoteAddress = string(endpoint); // local host
		driverHandler->ConnectionClose((uint8_t) fd); // precaution, TODO:: better handling
		if (driverHandler->UDPClientServerConnect(remoteAddress, (uint32_t) remote_port, (uint8_t) fd)){
			return 1;
		}
	}
	return 0;
}

int sendto_connection(char * buf, size_t length, uint32_t remote_address, uint16_t remote_port, int fd){
	string remoteAddress = "";
	char endpoint[20];
	if (driverHandler != NULL){
		uint8_t byte0 = remote_address & 0xFF;
		uint8_t byte1 = (remote_address >> 8) & 0xFF;
		uint8_t byte2 = (remote_address >> 16) & 0xFF;
		uint8_t byte3 = (remote_address >> 24) & 0xFF;
		snprintf(endpoint,20,"%d.%d.%d.%d",byte3, byte2, byte1, byte0);
		remoteAddress = string(endpoint); // local host
		if (driverHandler->UDPSendTo(buf, length,remoteAddress, remote_port, fd)){
			return 1;
		}
	}
	return 0;
}


int recvfrom_connection(char * buf, size_t length, uint32_t * remote_address, uint16_t * remote_port, int fd){
	char recv_address[20];
	uint16_t recv_port;
	int sockfd;
	int recv_length;
	if (driverHandler != NULL){
		recv_length = driverHandler->NBRecvFrom(recv_address, recv_port, buf, recv_length, sockfd);
		if (recv_length > 0){ // TODO:: check sockfd == fd
			//process IP
			(*remote_address) = convert_ipv4_to_number(recv_address);
			(*remote_port) = recv_port;
		}
	}
	return recv_length;
}

int get_rssi_indication(void){
	StationAccessPoint bearer;
	bearer = driverHandler->GetConnectedStationAP();
	return bearer.m_rssi;
}

char * get_local_ip(char* buffer){
	APDevicesList mySoftAP, myStationAP;
	driverHandler->GetLocalIPAddress(mySoftAP, myStationAP);
	strcpy(buffer,myStationAP.begin()->second.c_str());
	return buffer;
}

uint32_t get_local_ipv4(){
	char ipv4addr[20]; /* address i.e. "255.255.255.255"*/
	unsigned int byte3;
	unsigned int byte2;
	unsigned int byte1;
	unsigned int byte0;
	APDevicesList mySoftAP, myStationAP;
	uint32_t retval = -1;

	driverHandler->GetLocalIPAddress(mySoftAP, myStationAP);
	strcpy(ipv4addr,myStationAP.begin()->second.c_str());
	sscanf(ipv4addr, "%u.%u.%u.%u", &byte3, &byte2, &byte1, &byte0);
	if ((byte3 < 256) && (byte2 < 256) && (byte1 < 256) && (byte0 < 256)){
		retval = (byte3 << 24) + (byte2 << 16) + (byte1 << 8) +  byte0;
    }
	return retval;
}

uint32_t convert_ipv4_to_number(const char* buffer){
	unsigned int byte3;
	unsigned int byte2;
	unsigned int byte1;
	unsigned int byte0;
	uint32_t retval = -1;
	sscanf(buffer, "%u.%u.%u.%u", &byte3, &byte2, &byte1, &byte0);
	if ((byte3 < 256) && (byte2 < 256) && (byte1 < 256) && (byte0 < 256)){
		retval = (byte3 << 24) + (byte2 << 16) + (byte1 << 8) +  byte0;
    }
	return retval;
}

uint16_t convert_port_to_number(const char* buffer){
	int portvalue = -1;
	uint16_t retval = -1;
	sscanf(buffer, "%d", &portvalue);
	if ((portvalue >=0) && (portvalue <= 65536)){
		retval = (uint16_t) portvalue;
	}
	return retval;
}

char * get_local_mac(char* buffer){
	APDevicesList mySoftAP, myStationAP;
	driverHandler->GetLocalIPAddress(mySoftAP, myStationAP);
	strcpy(buffer,myStationAP.begin()->first.c_str());
	return buffer;
}

char * get_router_bssid(char* buffer){
	StationAccessPoint bearer;
	bearer = driverHandler->GetConnectedStationAP();
	strcpy(buffer,bearer.m_bssidmac.c_str());
	return buffer;
}

char * get_router_ssid(char* buffer){
	StationAccessPoint bearer;
	bearer = driverHandler->GetConnectedStationAP();
	strcpy(buffer,bearer.m_ssid.c_str());
	return buffer;
}

void get_connection_status(void){
	uint8_t clientserver;
	uint32_t localPort;
	uint32_t remotePort;
	string remoteIP = "";
    string conntype = "";
    uint8_t linkID;
	driverHandler->UpdateListActiveConnections(clientserver,localPort,remotePort,remoteIP,conntype,linkID);
}

int ping_server(const char * hostname){
	string remote =  string(hostname);
	if (driverHandler != NULL){
		if (driverHandler->Ping(remote)){
			return 1;
		}
	}
	return 0;
}
