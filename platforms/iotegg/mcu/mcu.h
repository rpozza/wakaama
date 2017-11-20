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

#ifndef MCU_H_
#define MCU_H_

#ifdef __cplusplus
extern "C" {
#endif

void reboot_mcu(void);
char * get_model_number(char * buffer);
char * get_serial_number(char * buffer);
char * get_part_id(char * buffer);
char * get_boot_code_version(char * buffer);
char * get_mbed_version(char * buffer);
void mcu_set_time(long timevalue);

bool has_watchdog_barked(void);
void watchdog_pet(void);
void watchdog_kick(int deadline);

#ifdef __cplusplus
}
#endif

#endif
