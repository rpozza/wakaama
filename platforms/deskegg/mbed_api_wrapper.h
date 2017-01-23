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

void reboot_mbed(void);
char * get_model_number(char * buffer);
char * get_serial_number(char * buffer);
char * get_part_id(char * buffer);
char * get_boot_code_version(char * buffer);
char * get_mbed_version(char * buffer);
void mbed_set_time(long timevalue);

//LED APIs
void init_rgb_leds(void);
void set_red(int rgbvalue, int dimming);
void set_green(int rgbvalue, int dimming);
void set_blue(int rgbvalue, int dimming);

//BUZZER APIs
void init_buzzer(void);
void attach_buzzer_on(unsigned int on_time, unsigned int off_time);
void detach_buzzer(void);
void set_buzzer_on(int dimming);
void set_buzzer_off(void);

//TEMPERATURE AND HUMIDITY APIs
void init_temp_humd(void);
unsigned int get_raw_temperature_cel(void);
unsigned int get_raw_humidity(void);

//GESTURE APIs
bool init_gesture_sensor(void);
void free_gesture_sensor(void);
void disable_gesture_irq(void);
void enable_gesture_irq(void);
bool is_gesture_isr_flag_set(void);
void reset_gesture_isr_flag(void);
void gesture_handler(void);
int get_last_gesture(void);

//AMBIENT API
int get_last_ambient_light(void);

//DUST APIs
void init_dust_sensor(void);
unsigned int sample_dust_adc(void);

//MIC APIs
void init_mic_sensor(void);
unsigned int sample_mic_adc(void);

//RANGE APIs
void init_ranging_sensor(void);
unsigned int sample_ranging_adc(void);

//FLASH MEMORY APIs
void init_ext_flash(void);
void erase_ext_flash(void);
bool flash_is_busy(void);
void flash_program_page(uint8_t * data, uint32_t address, int length);
void flash_read_page(uint8_t * data, uint32_t address, int length);

//WATCHDOG
bool has_watchdog_barked(void);
void watchdog_pet(void);
void watchdog_kick(int deadline);

#ifdef __cplusplus
}
#endif

#endif
