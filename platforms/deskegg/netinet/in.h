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

#ifndef NETINET_IN_H
#define NETINET_IN_H

#define IP_LAYER_DEBUG 0

#include <sys/socket.h>

typedef uint32_t in_addr_t;
struct in_addr{
	in_addr_t s_addr;
};

typedef uint16_t in_port_t;

struct in6_addr{
	union{
		uint8_t u6_addr8[16];
		uint16_t u6_addr16[8];
		uint32_t u6_addr32[4];
	} in6_u;
};

#define s6_addr 	in6_u.u6_addr8
#define s6_addr16   in6_u.u6_addr16
#define s6_addr32   in6_u.u6_addr32


#define INET_ADDRSTRLEN 16
#define INET6_ADDRSTRLEN 46

struct sockaddr_in{
	sa_family_t sin_family;
	in_port_t 	sin_port;
	struct in_addr sin_addr;
    /* Pad to size of `struct sockaddr'.  */
    unsigned char sin_zero[sizeof (struct sockaddr) -
						   sizeof (unsigned short int) -
                           sizeof (in_port_t) -
                           sizeof (struct in_addr)];
};

struct sockaddr_in6{
	sa_family_t sin6_family;
	in_port_t 	sin6_port;
	uint32_t sin6_flowinfo;     /* IPv6 flow information */
    struct in6_addr sin6_addr;  /* IPv6 address */
    uint32_t sin6_scope_id;     /* IPv6 scope-id */
};

# define IN6_IS_ADDR_V4MAPPED(a) \
   ({ const struct in6_addr *ina = (const struct in6_addr *) (a);       \
      ina->s6_addr32[0] == 0                                            \
      && ina->s6_addr32[1] == 0                                         \
      && ina->s6_addr32[2] == htonl (0xffff); })


uint32_t htonl (uint32_t hostlong);
uint16_t ntohs (uint16_t netshort);

#endif
