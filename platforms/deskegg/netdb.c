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
#include "esp8266_wrapper.h"

int getaddrinfo (const char *name, const char *service,const struct addrinfo *req, struct addrinfo **pai){
	struct addrinfo *ai;
	struct sockaddr_in *sa = NULL;
	int portValue;

	ai = (struct addrinfo*) malloc(sizeof(struct addrinfo));
	if (ai == NULL) {
		fprintf(stderr,"Error malloc addrinfo\n");
		return -1;
	}
	dbgprintf("[ a)getaddrinfo (AF=%d), (ST=%d)]\r\n",
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
				fprintf(stderr,"Error malloc sockaddr_in\n");
				return -1;
			}
			else{
				// clean socket address
				memset(sa, 0, sizeof(struct sockaddr_in));
				// if not connected yet, connects to access point
				connectESP8266toAP();
				sa->sin_family = AF_INET;
				sa->sin_addr.s_addr = get_local_ipv4();
				sa->sin_port = convert_port_to_number(service);
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
				fprintf(stderr,"Error memory allocation\n");
				return -1;
			}
			else{
				// clean socket address
				memset(sa, 0, sizeof(struct sockaddr_in));
				// if not connected yet, connects to access point
				connectESP8266toAP();
				if (ping_server(name) == 0){
					fprintf(stderr,"Error Destination Host Unreachable!!");
					return -1;
				}
				sa->sin_family = AF_INET;
				// NB: no domain host resolution!!
				sa->sin_addr.s_addr = convert_ipv4_to_number(name);
				sa->sin_port = convert_port_to_number(service);
				ai->ai_addr = (struct sockaddr *) sa;
			}
			ai->ai_next = NULL;
		}
		dbgprintf("[ b)getaddrinfo (SASF=%d), (SAAD=%X), (SAPT=%d) ]\r\n",
				sa->sin_family, sa->sin_addr.s_addr, sa->sin_port);
		*pai = ai;
		return 0;
	}
	// not supported
	return -1;
}

void freeaddrinfo(struct addrinfo *ai){

	dbgprintf("[freeaddrinfo (AF=%d)]\r\n",ai->ai_family);
	if (ai->ai_family == AF_INET){
		struct sockaddr_in * sa = (struct sockaddr_in *) ai->ai_addr;
		free(sa); // free sockaddr_in
	}
	free(ai);  // free addrinfo
}
