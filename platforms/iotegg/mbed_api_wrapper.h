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

#ifndef MBED_API_WRAPPER_H_
#define MBED_API_WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

//DUST APIs
void init_dust_sensor(void);
unsigned int sample_dust_adc(void);

//MIC APIs
void init_mic_sensor(void);
unsigned int sample_mic_adc(void);

//RANGE APIs
void init_ranging_sensor(void);
unsigned int sample_ranging_adc(void);


#ifdef __cplusplus
}
#endif

#endif
