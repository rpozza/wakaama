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

#ifndef SYS_SOCKET_H
#define SYS_SOCKET_H

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>
#include <arpa/inet.h>

typedef unsigned short int sa_family_t;

/* Address families.  */
#define AF_UNSPEC       0       /* Unspecified.  */
#define AF_INET			2       /* IP protocol family.  */
#define AF_INET6		10      /* IP version 6.  */

/* Types of sockets.  */
enum socket_type
{
  SOCK_STREAM = 1,              /* Sequenced, reliable, connection-based
                                   byte streams.  */
  SOCK_DGRAM = 2,               /* Connectionless, unreliable datagrams
                                   of fixed maximum length.  */
  SOCK_RAW = 3,                 /* Raw protocol interface.  */
};


/* Structure describing a generic socket address.  */
struct sockaddr{
	sa_family_t sa_family;	    /* Common data: address family and length.  */
    char sa_data[14];           /* Address data.  */
};

/* Structure large enough to hold any socket address (with the historical
   exception of AF_UNIX).  */
#define SS_PADSIZE  (128 - sizeof (unsigned short int) - sizeof (unsigned long int))

struct sockaddr_storage{
	sa_family_t ss_family;    		/* Address family, etc.  */
    char __ss_padding[SS_PADSIZE];
    unsigned long int __ss_align;  	/* Force desired alignment.  */
};

int socket (int domain, int type, int protocol);
int close(int fd);
int bind (int fd, const struct sockaddr *addr, socklen_t len);
int connect (int fd, const struct sockaddr *addr, socklen_t len);

/* Send N bytes of BUF on socket FD to peer at address ADDR (which is
   ADDR_LEN bytes long).  Returns the number sent, or -1 for errors.*/
ssize_t sendto (int fd, const void *buf, size_t n, int flags, struct sockaddr *addr,socklen_t addr_len);

/* Read N bytes into BUF through socket FD.
   If ADDR is not NULL, fill in *ADDR_LEN bytes of it with tha address of
   the sender, and store the actual size of the address in *ADDR_LEN.
   Returns the number of bytes read or -1 for errors.*/
ssize_t recvfrom (int fd, void *buf, size_t n, int flags, struct sockaddr *addr,socklen_t *addr_len);

#endif
