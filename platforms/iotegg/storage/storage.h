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

#ifndef STORAGE_H_
#define STORAGE_H_

#ifdef __cplusplus
extern "C" {
#endif

//LWM2M STORE ADDRESSES
#define LWM2M_MAGIC_CODE_ADDRESS     0x00000000	// first page
#define LWM2M_FOTA_URL_ADDRESS	 	 0x00000200 // third page
#define LWM2M_VAR_STORE_BASE_ADDRESS 0x00080000 // first 512KB reserved for binary files
#define LWM2M_VAR_STORE_PAGE_SIZE    0x1000	    // 4KB subsector (need to erase to 1 before writing again)
#define LWM2M_VAR_STORE_OFFSET		 0x100		// 2nd page of 4KB subsector
#define STORE_DEFAULT_VALUE			 0xFF		// 11111111b
#define STORE_DIRT_VALUE			 0xA5       // 10100101b

enum lwm2m_store_offset{ // add here (and in .cpp its value and format) new variables
	lifetime, min_period, max_period, utc_offset, rgb_colour, rgb_state, rgb_dimmer, loudness_calib, loudness_appl, dust_calib,
	dust_appl, range_calib, range_appl, buzzer_duration, buzzer_min_time_off, buzzer_appl, buzzer_state, buzzer_dimmer, gesture_appl,
	package_uri, fw_version, fw_package
};

//N25Q FLASH APIs
void init_ext_flash(void);
bool lwm2m_store_new_value(int offset, void *data, int length);
bool lwm2m_get_value(int offset, void *data, int length);
void lwm2m_store_boot(void);
void lwm2m_factory_default(void);
void firmware_ota_update(char *uri, int length);
void ota_restore(void);

#ifdef __cplusplus
}
#endif

#endif
