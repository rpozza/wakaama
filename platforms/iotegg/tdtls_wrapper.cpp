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

#include "rtos.h"
#include "tdtls_wrapper.h"

//static Mutex CypherMutex;

void pthread_mutex_lock(pthread_mutex_t *){
//	printf("CypherMutex LOCK!\r\n");
//	CypherMutex.lock();
}

void pthread_mutex_unlock(pthread_mutex_t *){
//	printf("CypherMutex UNLOCK!\r\n");
//	CypherMutex.unlock();
}
