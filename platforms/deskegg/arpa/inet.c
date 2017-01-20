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

#include <arpa/inet.h>
#include <netinet/in.h>


const char *inet_ntop (int af, const void *cp, char *buf, socklen_t len){
	unsigned char byte3;
	unsigned char byte2;
	unsigned char byte1;
	unsigned char byte0;

	dbgprintf("[ inet_ntop (AF=%d), ",af);
	if (af == AF_INET){
		//ipv4
		struct in_addr *sa = (struct in_addr *) cp;
		byte0 = sa->s_addr & 0xFF;
		byte1 = (sa->s_addr >> 8) & 0xFF;
		byte2 = (sa->s_addr >> 16) & 0xFF;
		byte3 = (sa->s_addr >> 24) & 0xFF;
		snprintf(buf,len,"%d.%d.%d.%d",byte3, byte2, byte1, byte0);
		dbgprintf("inet_ntop %.*s] \r\n",len,buf);
	}
	return buf;
}
