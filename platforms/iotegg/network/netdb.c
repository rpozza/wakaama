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
#include <stdio.h>
#include <netdb.h>
#include "mbed_debug.h"
#include "ESP8266InterfaceCWrapper.h"

int getaddrinfo (const char *name, const char *service,const struct addrinfo *req, struct addrinfo **pai){
	struct addrinfo *ai;
	struct sockaddr_in *sa = NULL;
	int portValue;

	ai = (struct addrinfo*) malloc(sizeof(struct addrinfo));
	if (ai == NULL) {
		debug_if(IP_LAYER_DEBUG,"IP> ALLOCATION ERROR\r\n");
		return -1;
	}
	debug_if(IP_LAYER_DEBUG,"IP> GETADDRINFO (AF=%d), (ST=%d)\r\n",
					req->ai_family, req->ai_socktype);
	//UDP, IPv4
	if ((req->ai_socktype == SOCK_DGRAM) && (req->ai_family == AF_INET)){
		//localhost, binding socket accepting connections
		if ((name == NULL) && (req->ai_flags == AI_PASSIVE)){
			// prepares data structure for address
			ai->ai_flags = req->ai_flags;
			ai->ai_family = req->ai_family;
			ai->ai_socktype = req->ai_socktype;
			ai->ai_protocol = 0;
			ai->ai_addrlen = sizeof (struct sockaddr_in);
			sa = (struct sockaddr_in *) malloc (sizeof (struct sockaddr_in));
			if (sa == NULL) {
				debug_if(IP_LAYER_DEBUG,"IP> ALLOCATION ERROR\r\n");
				return -1;
			}
			else{
				// clean socket address
				memset(sa, 0, sizeof(struct sockaddr_in));
				// if not connected yet, connects to access point
				if (isEsp8266IfDown()){
					while (!turnEsp8266IfUp());
				}
				if (!isEsp8266APconnected()){
					while (!connectToAP());
				}
				// needed to be sure the get local_ip works
				sa->sin_family = AF_INET;
				sa->sin_addr.s_addr = toU32_IPv4(getIPAddress());
				sa->sin_port = toU16_port(service);
				ai->ai_addr = (struct sockaddr *) sa;
			}
			ai->ai_next = NULL;
		}
		//remote host configuration
		else{
			ai->ai_family = req->ai_family; // still IPv4
			ai->ai_socktype = req->ai_socktype;
			ai->ai_addrlen = sizeof (struct sockaddr_in);
			ai->ai_protocol = 0;
			sa = (struct sockaddr_in *) malloc (sizeof (struct sockaddr_in));
			if (sa == NULL) {
				debug_if(IP_LAYER_DEBUG,"IP> ALLOCATION ERROR\r\n");
				return -1;
			}
			else{
				// clean socket address
				memset(sa, 0, sizeof(struct sockaddr_in));
				// if not connected yet, connects to access point
				if (isEsp8266IfDown()){
					debug_if(IP_LAYER_DEBUG,"IP> TURNING UP IF\r\n");
					while (!turnEsp8266IfUp());
				}
				if (!isEsp8266APconnected()){
					debug_if(IP_LAYER_DEBUG,"IP> CONNECT TO AP\r\n");
					while (!connectToAP());
				}
				if (ping(name) < 0){
					debug_if(IP_LAYER_DEBUG,"IP> GETADDRINFO HOST UNREACHABLE\r\n");
				}
				sa->sin_family = AF_INET;
				// TODO: domain host resolution!!
				sa->sin_addr.s_addr = toU32_IPv4(name);
				sa->sin_port = toU16_port(service);
				ai->ai_addr = (struct sockaddr *) sa;
			}
			ai->ai_next = NULL;
		}
		debug_if(IP_LAYER_DEBUG,"IP> GETADDRINFO (SASF=%d), (SAAD=%X), (SAPT=%d)\r\n",
				sa->sin_family, sa->sin_addr.s_addr, sa->sin_port);
		*pai = ai;
		return 0;
	}
	// not supported
	return -1;
}

void freeaddrinfo(struct addrinfo *ai){

	debug_if(IP_LAYER_DEBUG,"IP> FREEADDRINFO (AF=%d)\r\n",ai->ai_family);
	if (ai->ai_family == AF_INET){
		struct sockaddr_in * sa = (struct sockaddr_in *) ai->ai_addr;
		free(sa); // free sockaddr_in
	}
	free(ai);  // free addrinfo
}
