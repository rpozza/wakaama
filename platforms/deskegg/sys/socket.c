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
#include "esp8266_wrapper.h"

#ifdef WITH_TINYDTLS
#include "dtlsconnection.h"
#else
#include "connection.h"
#endif

typedef struct _sockdesc_t{
	int sockfamily;
	int socktype;
	int sockprotocol;
} sockdesc_t;

static sockdesc_t socketdescriptions[5];
static int socketisopen[5] = { 0 };
static int reboot_socket_family =0;
static uint16_t reboot_socket_port =0;
static int local_sockfd = 0;
static int retriesBeforeReboot = 0; // initialization value = null

int socket (int domain, int type, int protocol){
	int i;
	for (i=0; i < 5; i++){
		if (socketisopen[i] == 0){
			socketisopen[i] = 1;
			socketdescriptions[i].sockfamily = domain;
			socketdescriptions[i].socktype = type;
			socketdescriptions[i].sockprotocol = protocol;
			dbgprintf("[ socket (%d) (AF=%d), (ST=%d), (PT=%d)]\r\n", i, domain, type, protocol);
			return i;
		}
	}
	fprintf(stderr,"too many opened sockets");
	return -1;
}

int  close(int fd){
	dbgprintf("[ close (%d) ]\r\n", fd);
	socketisopen[fd] = 0;
	socketdescriptions[fd].sockfamily = 0;
	socketdescriptions[fd].socktype = 0;
	socketdescriptions[fd].sockprotocol = 0;
	if (close_connection(fd) == 1){
		return 0;
	}
	return -1;
}

int bind (int fd, const struct sockaddr *addr, socklen_t len){
	if (socketdescriptions[fd].sockfamily == AF_INET){
		struct sockaddr_in * localsa = (struct sockaddr_in *) addr;
		dbgprintf("[ bind (%d) (ADD=%X), (PORT=%d)]\r\n", fd, localsa->sin_addr.s_addr, localsa->sin_port);
		if (bind_connection(fd,localsa->sin_addr.s_addr,localsa->sin_port) == 1){
			reboot_socket_port = localsa->sin_port;
			reboot_socket_family = AF_INET;
			local_sockfd = fd;
			return fd;
		}
	}
	return -1;
}

int connect (int fd, const struct sockaddr *addr, socklen_t len){
	if (socketdescriptions[fd].sockfamily == AF_INET){
		struct sockaddr_in * remotesa = (struct sockaddr_in *) addr;
		dbgprintf("[ connect (%d) (ADD=%X), (PORT=%d)]\r\n", fd, remotesa->sin_addr.s_addr, remotesa->sin_port);
		if (connect_connection(fd,remotesa->sin_addr.s_addr,remotesa->sin_port) == 1){
			return fd;
		}
	}
	return -1;
}

ssize_t sendto (int fd, const void *buf, size_t n, int flags, struct sockaddr *addr, socklen_t addr_len){
	ssize_t length = n;
	if (n >= 2047){
		length = 2047; // driver would allow up to 2048 bytes
	}
	if (socketdescriptions[fd].sockfamily == AF_INET){
		struct sockaddr_in * remotesa = (struct sockaddr_in *) addr;
		dbgprintf("[ send (%d) (ADD=%X), (PORT=%d)]\r\n", fd, remotesa->sin_addr.s_addr, remotesa->sin_port);
		if (sendto_connection((char *) buf, length, remotesa->sin_addr.s_addr, remotesa->sin_port, fd) == 1){
			retriesBeforeReboot = 0;
			return length;
		}
		else{
			// a timeout or error in send has been received
			retriesBeforeReboot++;
			fprintf(stderr,"Try again %d\r\n", (retriesBeforeReboot));
			if (retriesBeforeReboot >= 2){
				retriesBeforeReboot = 0;
				fprintf(stderr,"Rebooting ESP8266!!\r\n");
				rebootESP8266();
				//now need to virtually "close" the lost socket and reopen a new one
				socketisopen[local_sockfd] = 0;
				socketdescriptions[local_sockfd].sockfamily = 0;
				socketdescriptions[local_sockfd].socktype = 0;
				socketdescriptions[local_sockfd].sockprotocol = 0;
				char portstr[10];
				snprintf(portstr,10,"%d",reboot_socket_port);
				create_socket(portstr, reboot_socket_family);
				fprintf(stderr,"Rebooted!!\r\n");
			}
//			get_connection_status();
//			char bufera[20];
//			get_local_ip(bufera);
//			get_router_bssid(bufera);
//			ping_server("8.8.8.8");
		}
		fprintf(stderr,"Error %d bytes sent\r\n", length);
		return -1;
	}
	fprintf(stderr,"No Ipv4!\r\n");
	return -1;
}

ssize_t recvfrom (int fd, void *buf, size_t n, int flags, struct sockaddr *addr,socklen_t *addr_len){
	size_t bytesReceived;
	uint32_t remote_ip;
	uint16_t remote_port;
	struct sockaddr_in * sa = (struct sockaddr_in *) addr;

	bytesReceived = recvfrom_connection((char *) buf, n, &remote_ip, &remote_port, fd);
	if (bytesReceived > 0){// there's actually some packet
		dbgprintf("[ recv (%d) (ADD=%X), (PORT=%d)]\r\n", fd, remote_ip, remote_port);
		if (socketdescriptions[fd].sockfamily == AF_INET){
			memset(sa, 0, sizeof(struct sockaddr_in));
			sa->sin_family = AF_INET;
			sa->sin_addr.s_addr = remote_ip;
			sa->sin_port = remote_port;
			*addr_len = sizeof (struct sockaddr_in);
		}
		if (bytesReceived > n){ // if packet > 1024 bytes
			fprintf(stderr,"\r\nWarning!! Buffer overflow!!!!!!\r\n");
		}
	}
	return bytesReceived;
}
