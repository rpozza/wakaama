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

#include <netinet/in.h>
#include "mbed_debug.h"
#include "ESP8266InterfaceCWrapper.h"

#ifdef WITH_TINYDTLS
#include "dtlsconnection.h"
#else
#include "connection.h"
#endif

#define ESP8266_MAX_NUMBER_SOCKETS 	 		5

typedef struct _sockdesc_t{
	int sockfamily;
	int socktype;
	int sockprotocol;
} sockdesc_t;

static sockdesc_t sockDesc[ESP8266_MAX_NUMBER_SOCKETS];
static int sockID[ESP8266_MAX_NUMBER_SOCKETS] = { 0 };

typedef struct _reboot_info_t{
	sa_family_t sa_family;
	in_port_t 	sin_port;
	int fd;
} reboot_info_t;

static reboot_info_t rebootLocal = {0,0,0}; // initialization value = null
static int retriesBeforeReboot = 0; // initialization value = null

int socket (int domain, int type, int protocol){
	int i;
	for (i=0; i < ESP8266_MAX_NUMBER_SOCKETS; i++){
		if (sockID[i] == 0){
			sockID[i] = 1;
			sockDesc[i].sockfamily = domain;
			sockDesc[i].socktype = type;
			sockDesc[i].sockprotocol = protocol;
			debug_if(IP_LAYER_DEBUG,"IP> NEW SOCKET (%d) (AF=%d), (ST=%d), (PT=%d)\r\n", i, domain, type, protocol);
			return i;
		}
	}
	debug_if(IP_LAYER_DEBUG,"IP> ERROR MAX SOCKETS (>%d)\r\n", ESP8266_MAX_NUMBER_SOCKETS);
	return -1;
}

int  close(int fd){
	debug_if(IP_LAYER_DEBUG,"IP> CLOSING SOCKET (%d)\r\n", fd);
	if (esp8266close(fd)){
		sockID[fd] = 0;
		sockDesc[fd].sockfamily = 0;
		sockDesc[fd].socktype = 0;
		sockDesc[fd].sockprotocol = 0;
		return 0; //OK
	}
	debug_if(IP_LAYER_DEBUG,"IP> WARNING FAILED CLOSING SOCKET (%d)\r\n", fd);
	//TODO:: not sure here, still deallocating the socket descriptors and removing it
	sockID[fd] = 0;
	sockDesc[fd].sockfamily = 0;
	sockDesc[fd].socktype = 0;
	sockDesc[fd].sockprotocol = 0;
	return -1;
}

int bind (int fd, const struct sockaddr *addr, socklen_t len){
	char bind_ipadd[INET_ADDRSTRLEN];
	in_port_t bind_port;

	if (sockDesc[fd].sockfamily == AF_INET){
		struct sockaddr_in * localsa = (struct sockaddr_in *) addr;
		inet_ntop(localsa->sin_family, &localsa->sin_addr, bind_ipadd, INET_ADDRSTRLEN);
		bind_port = localsa->sin_port;
		debug_if(IP_LAYER_DEBUG,"IP> BINDING SOCKET (%d) TO (ADDR=%s), (PORT=%d)\r\n", fd, bind_ipadd, bind_port);

		if (esp8266bind(fd, bind_ipadd, (int) bind_port)){
			debug_if(IP_LAYER_DEBUG,"IP> BOUND TO SOCKET (%d)\r\n",fd);
			//save here state of bound connection for module reboot
			rebootLocal.sa_family = AF_INET;
			rebootLocal.sin_port = localsa->sin_port;
			rebootLocal.fd = fd;
			return fd;
		}
		debug_if(IP_LAYER_DEBUG,"IP> FAILED BINDING TO SOCKET (%d)\r\n",fd);
		return -1;
	}
	//TODO:: handle AF_INET6, but not supported by HW
	debug_if(IP_LAYER_DEBUG,"IP> NO SOCKET / IPV6 ADDRESS NOT SUPPORTED\r\n",fd);
	return -1;
}

int connect (int fd, const struct sockaddr *addr, socklen_t len){
	char connect_ipadd[INET_ADDRSTRLEN];
	in_port_t connect_port;

	if (sockDesc[fd].sockfamily == AF_INET){
		struct sockaddr_in * remotesa = (struct sockaddr_in *) addr;
		inet_ntop(remotesa->sin_family, &remotesa->sin_addr, connect_ipadd, INET_ADDRSTRLEN);
		connect_port = remotesa->sin_port;
		debug_if(IP_LAYER_DEBUG,"IP> CONNECTING SOCKET (%d) TO (ADDR=%s), (PORT=%d)\r\n", fd, connect_ipadd, connect_port);

		if (esp8266connect("UDP",fd, connect_ipadd, (int) connect_port)){
			debug_if(IP_LAYER_DEBUG,"IP> CONNECTED TO SOCKET (%d)\r\n",fd);
			return fd;
		}
		debug_if(IP_LAYER_DEBUG,"IP> FAILED CONNECTING TO SOCKET (%d)\r\n",fd);
		return -1;
	}
	//TODO:: handle AF_INET6, but not supported by HW
	debug_if(IP_LAYER_DEBUG,"IP> NO SOCKET / IPV6 ADDRESS NOT SUPPORTED\r\n",fd);
	return -1;
}

ssize_t sendto (int fd, const void *buf, size_t n, int flags, struct sockaddr *addr, socklen_t addr_len){
	char send_ipadd[INET_ADDRSTRLEN];
	in_port_t send_port;
	char portstr[6];
	ssize_t actualLength = n;
	if (n >= 2047){
		// esp theoretically would allow up to 2048 bytes
		actualLength = 2047;
	}
	if (sockDesc[fd].sockfamily == AF_INET){
		struct sockaddr_in * remotesa = (struct sockaddr_in *) addr;
		inet_ntop(remotesa->sin_family, &remotesa->sin_addr, send_ipadd, INET_ADDRSTRLEN);
		send_port = remotesa->sin_port;
		debug_if(IP_LAYER_DEBUG,"IP> SENDING VIA SOCKET (%d) TO (ADDR=%s), (PORT=%d)\r\n", fd, send_ipadd, send_port);

		if (esp8266sendto(fd, buf, actualLength, send_ipadd, send_port)){
			debug_if(IP_LAYER_DEBUG,"IP> SENT %d BYTES VIA SOCKET (%d)\r\n", actualLength, fd);
			retriesBeforeReboot = 0;
			return actualLength;
		}
		else{
			retriesBeforeReboot++;
			debug_if(IP_LAYER_DEBUG,"IP> FAIL SENDING TENTATIVE %d \r\n", retriesBeforeReboot);
			if (retriesBeforeReboot >= 3){ //NB:: more than 3(*2)consecutive packet transmission errors!
				// hacking mode on..
				retriesBeforeReboot = 0;
				debug_if(true,"Rebooting1!.. ");
				debug_if(IP_LAYER_DEBUG,"IP> POWER CYCLE ESP8266\r\n");
				while(!powerCycleESP8266()); // try to reconnect to Access Point hopefully before watchdog enters a reboot
				// virtually "close" the lost socket (already closed by power cycle)
				sockID[rebootLocal.fd] = 0;
				sockDesc[rebootLocal.fd].sockfamily = 0;
				sockDesc[rebootLocal.fd].socktype = 0;
				sockDesc[rebootLocal.fd].sockprotocol = 0;
				// reopenup via connection
				snprintf(portstr,6,"%d",rebootLocal.sin_port);
				create_socket(portstr, rebootLocal.sa_family);
				debug_if(true,"Rebooted1!\r\n");
				debug_if(IP_LAYER_DEBUG,"IP> REBOOTED ESP8266!\r\n");
			}
		}
		debug_if(IP_LAYER_DEBUG,"IP> FAIL SENDING %d BYTES\r\n", actualLength);
		return -1;
	}
	//TODO:: handle AF_INET6, but not supported by HW
	debug_if(true,"Rebooting2!.. ");
	debug_if(IP_LAYER_DEBUG,"IP> NO SOCKET? / IPV6 ADDRESS NOT SUPPORTED\r\n",fd);
	debug_if(IP_LAYER_DEBUG,"IP> POWER CYCLE ESP8266\r\n");
	while(!powerCycleESP8266()); // try to reconnect to Access Point hopefully before watchdog enters a reboot
	// virtually "close" the lost socket (already closed by power cycle)
	sockID[rebootLocal.fd] = 0;
	sockDesc[rebootLocal.fd].sockfamily = 0;
	sockDesc[rebootLocal.fd].socktype = 0;
	sockDesc[rebootLocal.fd].sockprotocol = 0;
	// reopenup via connection
	snprintf(portstr,6,"%d",rebootLocal.sin_port);
	create_socket(portstr, rebootLocal.sa_family);
	debug_if(true,"Rebooted2!\r\n");
	debug_if(IP_LAYER_DEBUG,"IP> REBOOTED ESP8266!\r\n");
	return -1;
}

ssize_t recvfrom (int fd, void *buf, size_t n, int flags, struct sockaddr *addr,socklen_t *addr_len){
	int32_t bytesReceived;
	char remote_ip[INET_ADDRSTRLEN];
	int remote_port;

	struct sockaddr_in * sa = (struct sockaddr_in *) addr;

	bytesReceived = esp8266recvfrom(fd, remote_ip, &remote_port, buf, n);
	//bytesReceived either > 0 or equal to -1
	if (bytesReceived > 0){// no error
		debug_if(IP_LAYER_DEBUG,"IP> RECEIVED %d BYTES FROM (ADD=%s), (PORT=%d)\r\n", bytesReceived, remote_ip, remote_port);
		if (bytesReceived > n){ // if packet > 1024 bytes
			debug_if(IP_LAYER_DEBUG,"IP> WARNING BUFFER OVERFLOW\r\n");
			return -1;
		}
		if (sockDesc[fd].sockfamily == AF_INET){ //not needed
			memset(sa, 0, sizeof(struct sockaddr_in));
			sa->sin_family = AF_INET;
			sa->sin_addr.s_addr = toU32_IPv4(remote_ip);
			sa->sin_port = (in_port_t) remote_port;
			*addr_len = sizeof (struct sockaddr_in);
			return bytesReceived;
		}
	}
	return 0;
}
