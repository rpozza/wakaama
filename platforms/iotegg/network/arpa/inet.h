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

#ifndef ARPA_INET_H
#define ARPA_INET_H

#include <sys/types.h>
typedef uint32_t socklen_t;

/* Convert a Internet address in binary network format for interface
   type AF in buffer starting at CP to presentation form and place
   result in buffer of length LEN astarting at BUF.  */
const char *inet_ntop (int af, const void *cp, char *buf, socklen_t len);

#endif
