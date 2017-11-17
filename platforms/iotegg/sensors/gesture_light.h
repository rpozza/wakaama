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

#ifndef GESTURE_LIGHT_H_
#define GESTURE_LIGHT_H_

#ifdef __cplusplus
extern "C" {
#endif

//GESTURE APIs
bool init_gesture_sensor(void);
void free_gesture_sensor(void);
void disable_gesture_irq(void);
void enable_gesture_irq(void);
bool is_gesture_isr_flag_set(void);
void reset_gesture_isr_flag(void);
void gesture_handler(void);
int get_last_gesture(void);

//LIGHT APIs
int get_last_ambient_light(void);


#ifdef __cplusplus
}
#endif

#endif
