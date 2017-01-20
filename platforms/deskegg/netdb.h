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

#ifndef NETDB_H
#define NETDB_H

/* Structure to contain information about address of a service provider.  */
struct addrinfo
{
  int ai_flags;                 /* Input flags.  */
  int ai_family;                /* Protocol family for socket.  */
  int ai_socktype;              /* Socket type.  */
  int ai_protocol;              /* Protocol for socket.  */
  socklen_t ai_addrlen;         /* Length of socket address.  */
  struct sockaddr *ai_addr;     /* Socket address for socket.  */
  char *ai_canonname;           /* Canonical name for service location.  */
  struct addrinfo *ai_next;     /* Pointer to next in list.  */
};

/* Possible values for `ai_flags' field in `addrinfo' structure.  */
#define AI_PASSIVE     0x0001  /* Socket address is intended for `bind'.  */
#define AI_CANONNAME   0x0002  /* Request for canonical name.  */
#define AI_NUMERICHOST 0x0004  /* Don't use name resolution.  */
#define AI_V4MAPPED    0x0008  /* IPv4 mapped addresses are acceptable.  */
#define AI_ALL         0x0010  /* Return IPv4 mapped and IPv6 addresses.  */
#define AI_ADDRCONFIG  0x0020  /* Use configuration of this host to choose
                                   returned address type..  */

/* Translate name of a service location and/or a service(port) name(host) to set of
   socket addresses.*/
int getaddrinfo (const char *name, const char *service,const struct addrinfo *req, struct addrinfo **pai);

/* Free `addrinfo' structure AI including associated storage.  */
void freeaddrinfo (struct addrinfo *ai);


#endif
