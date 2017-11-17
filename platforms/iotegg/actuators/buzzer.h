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

#ifndef BUZZER_H_
#define BUZZER_H_

#ifdef __cplusplus
extern "C" {
#endif

//BUZZER APIs
void init_buzzer(void);
void attach_buzzer_on(unsigned int on_time, unsigned int off_time);
void detach_buzzer(void);
void set_buzzer_on(int dimming);
void set_buzzer_off(void);

#ifdef __cplusplus
}
#endif

#endif
