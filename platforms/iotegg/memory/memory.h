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

#ifndef MEMORY_H_
#define MEMORY_H_

#ifdef __cplusplus
extern "C" {
#endif

//FLASH MEMORY APIs
void init_ext_flash(void);
void erase_ext_flash(void);
bool flash_is_busy(void);
void flash_program_page(uint8_t * data, uint32_t address, int length);
void flash_read_page(uint8_t * data, uint32_t address, int length);
void flash_write_blocking(uint8_t * data, uint32_t address, int length);
void flash_read_blocking(uint8_t * data, uint32_t address, int length);
void default_ext_flash_values(void);
void serialize_uint32_t (uint32_t value, uint32_t address);
uint32_t deserialize_uint32_t (uint32_t address);
void serialize_char_t (char * value, int len, uint32_t address);
void deserialize_char_t (char * value, uint32_t address, int len);
void serialize_bool_t (bool value, uint32_t address);
bool deserialize_bool_t (uint32_t address);
void serialize_uint8_t (uint8_t value, uint32_t address);
uint8_t deserialize_uint8_t (uint32_t address);

void flash_subsector_4K_erase(uint32_t address);
void flash_sector_64K_erase(uint32_t address);

//SBL //TODO::REPLACE WITH URI HTTP DOWNLOAD AT BOOT
void flash_secondary_boot_loader(void);

#ifdef __cplusplus
}
#endif

#endif
