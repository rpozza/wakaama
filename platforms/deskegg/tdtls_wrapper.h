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

#ifndef TDTLS_WRAPPER_H_
#define TDTLS_WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#define PTHREAD_MUTEX_INITIALIZER 0
typedef int pthread_mutex_t; // fooling the tdtls interface


void pthread_mutex_lock(pthread_mutex_t *);

void pthread_mutex_unlock(pthread_mutex_t *);


#ifdef __cplusplus
}
#endif

#endif
