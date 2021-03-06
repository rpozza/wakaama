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

#include "mbed.h"
#include "n25q.h"
#include "storage.h"

//LWM2M STORE VALUES
const uint32_t def_lifetime = 300;
const uint32_t def_min_period = 0;
const uint32_t def_max_period = 1;
const char def_utc_offset[] = "+00:00";
const char def_rgb_colour[] = "#000000";
const bool def_rgb_state = false;
const uint8_t def_rgb_dimmer = 100;
const double def_loudness_calib = 0;
const char def_loudness_appl[] = "Loudness Sensor";
const double def_dust_calib = (0.5 / 2.8);
const char def_dust_appl[] = "Dust Sensor";
const double def_range_calib = 0;
const char def_range_appl[] = "Ranging Sensor";
const double def_buzzer_duration = 0;
const double def_buzzer_min_time_off = 0;
const char def_buzzer_appl[] = "Buzzer/Alarm";
const bool def_buzzer_state = false;
const uint8_t def_buzzer_dimmer = 100;
const char def_gesture_appl[] = "Gesture Sensor";
const char def_package_uri[]  = "http://example.org/";
const char def_fw_version[]  = "v2.3.0";
const char def_fw_package[]  = "IoTEgg";

//MAGIC CODE
const char magic_code[] = MAGIC_CODE;

static N25Q * N25Q_Driver = NULL;

void init_ext_flash(void){
#if defined(TARGET_ARCH_PRO)
	if (N25Q_Driver == NULL){
		N25Q_Driver = new N25Q();
	}
#endif
}

bool lwm2m_store_is_default(int offset){
	uint8_t flag;
	if (N25Q_Driver != NULL){
		int store_address = (int) LWM2M_VAR_STORE_BASE_ADDRESS + ((int) LWM2M_VAR_STORE_PAGE_SIZE * offset);
		if(1 == N25Q_Driver->ReadFrom(&flag,store_address,1)){
			if (flag == STORE_DEFAULT_VALUE){
				return true;
			}
		}
	}
	return false;
}

bool lwm2m_store_new_value(int offset, void *data, int length){
	uint8_t flag = STORE_DIRT_VALUE;
	if (N25Q_Driver != NULL){
		int store_address = (int) LWM2M_VAR_STORE_BASE_ADDRESS + ((int) LWM2M_VAR_STORE_PAGE_SIZE * offset);
		if(N25Q_Driver->SubSectorErase(store_address)){
			//cleanup, then
			if(1 == N25Q_Driver->ProgramPageFrom(&flag,store_address,1)){
				//made dirt, and finally store data in next page
				store_address += (int) LWM2M_VAR_STORE_OFFSET;
				if(length == N25Q_Driver->ProgramPageFrom(data,store_address,length)){
					return true;
				}
			}
		}
	}
	return false;
}

bool lwm2m_get_value(int offset, void *data, int length){
	if (N25Q_Driver != NULL){
		int store_address = (int) LWM2M_VAR_STORE_BASE_ADDRESS + ((int) LWM2M_VAR_STORE_PAGE_SIZE * offset);
		store_address += (int) LWM2M_VAR_STORE_OFFSET;
		if(length == (N25Q_Driver->ReadFrom(data,store_address,length))){
			return true;
		}
	}
	return false;
}

void lwm2m_store_boot(void){
	ota_restore();

	if (lwm2m_store_is_default(lifetime)){
		lwm2m_store_new_value(lifetime,(void*) &def_lifetime, sizeof(def_lifetime));
	}
	if (lwm2m_store_is_default(min_period)){
		lwm2m_store_new_value(min_period,(void*) &def_min_period, sizeof(def_min_period));
	}
	if (lwm2m_store_is_default(max_period)){
		lwm2m_store_new_value(max_period,(void*) &def_max_period, sizeof(def_max_period));
	}
	if (lwm2m_store_is_default(utc_offset)){
		lwm2m_store_new_value(utc_offset,(void*) def_utc_offset, sizeof(def_utc_offset));
	}
	if (lwm2m_store_is_default(rgb_colour)){
		lwm2m_store_new_value(rgb_colour,(void*) def_rgb_colour, sizeof(def_rgb_colour));
	}
	if (lwm2m_store_is_default(rgb_state)){
		lwm2m_store_new_value(rgb_state,(void*) &def_rgb_state, sizeof(def_rgb_state));
	}
	if (lwm2m_store_is_default(rgb_dimmer)){
		lwm2m_store_new_value(rgb_dimmer,(void*) &def_rgb_dimmer, sizeof(def_rgb_dimmer));
	}
	if (lwm2m_store_is_default(loudness_calib)){
		lwm2m_store_new_value(loudness_calib,(void*) &def_loudness_calib, sizeof(def_loudness_calib));
	}
	if (lwm2m_store_is_default(loudness_appl)){
		lwm2m_store_new_value(loudness_appl,(void*) def_loudness_appl, sizeof(def_loudness_appl));
	}
	if (lwm2m_store_is_default(dust_calib)){
		lwm2m_store_new_value(dust_calib,(void*) &def_dust_calib, sizeof(def_dust_calib));
	}
	if (lwm2m_store_is_default(dust_appl)){
		lwm2m_store_new_value(dust_appl,(void*) def_dust_appl, sizeof(def_dust_appl));
	}
	if (lwm2m_store_is_default(range_calib)){
		lwm2m_store_new_value(range_calib,(void*) &def_range_calib, sizeof(def_range_calib));
	}
	if (lwm2m_store_is_default(range_appl)){
		lwm2m_store_new_value(range_appl,(void*) def_range_appl, sizeof(def_range_appl));
	}
	if (lwm2m_store_is_default(buzzer_duration)){
		lwm2m_store_new_value(buzzer_duration,(void*) &def_buzzer_duration, sizeof(def_buzzer_duration));
	}
	if (lwm2m_store_is_default(buzzer_min_time_off)){
		lwm2m_store_new_value(buzzer_min_time_off,(void*) &def_buzzer_min_time_off, sizeof(def_buzzer_min_time_off));
	}
	if (lwm2m_store_is_default(buzzer_appl)){
		lwm2m_store_new_value(buzzer_appl,(void*) def_buzzer_appl, sizeof(def_buzzer_appl));
	}
	if (lwm2m_store_is_default(buzzer_state)){
		lwm2m_store_new_value(buzzer_state,(void*) &def_buzzer_state, sizeof(def_buzzer_state));
	}
	if (lwm2m_store_is_default(buzzer_dimmer)){
		lwm2m_store_new_value(buzzer_dimmer,(void*) &def_buzzer_dimmer, sizeof(def_buzzer_dimmer));
	}
	if (lwm2m_store_is_default(gesture_appl)){
		lwm2m_store_new_value(gesture_appl,(void*) def_gesture_appl, sizeof(def_gesture_appl));
	}
	if (lwm2m_store_is_default(package_uri)){
		lwm2m_store_new_value(package_uri,(void*) def_package_uri, sizeof(def_package_uri));
	}
	if (lwm2m_store_is_default(fw_version)){
		lwm2m_store_new_value(fw_version,(void*) def_fw_version, sizeof(def_fw_version));
	}
	if (lwm2m_store_is_default(fw_package)){
		lwm2m_store_new_value(fw_package,(void*) def_fw_package, sizeof(def_fw_package));
	}

}

void lwm2m_factory_default(void){
	int store_address;
	if (N25Q_Driver != NULL){
		for (int i=lifetime; i<=fw_package; i++){
			store_address = (int) LWM2M_VAR_STORE_BASE_ADDRESS + ((int) LWM2M_VAR_STORE_PAGE_SIZE * i);
			N25Q_Driver->SubSectorErase(store_address);
		}
	}
}

void firmware_ota_update(char *uri, int length){
	uint8_t flag = STORE_DIRT_VALUE;
	if (N25Q_Driver != NULL){
		int store_address = (int) LWM2M_MAGIC_CODE_ADDRESS;
		if(N25Q_Driver->SubSectorErase(store_address)){
			//cleanup, then
			if(1 == N25Q_Driver->ProgramPageFrom(&flag,store_address,1)){
				//made dirt, and finally store data in next page
				store_address += (int) LWM2M_VAR_STORE_OFFSET;
				if((sizeof(magic_code)) == (N25Q_Driver->ProgramPageFrom((void*) magic_code,store_address,sizeof(magic_code)))){
					//write magic code, then
					store_address = (int) LWM2M_FOTA_URL_ADDRESS;
					if(1 == N25Q_Driver->ProgramPageFrom(&flag,store_address,1)){
						//made dirt, and finally store data in next page
						store_address += (int) LWM2M_VAR_STORE_OFFSET;
						if(length == N25Q_Driver->ProgramPageFrom((void*) uri,store_address,length)){
							// write url and overwrite the url to avoid spoofing
							lwm2m_store_new_value(package_uri,(void*) def_package_uri, sizeof(def_package_uri));
						}
					}
				}
			}
		}
	}
}

void ota_restore(void){
	uint8_t flag;
	if (N25Q_Driver != NULL){
		int store_address = (int) LWM2M_MAGIC_CODE_ADDRESS;
		if(1 == N25Q_Driver->ReadFrom(&flag,store_address,1)){
			if (flag == STORE_DIRT_VALUE){
				if(N25Q_Driver->SubSectorErase(store_address)){
					//dirt, then cleanup magic code (also cleans url together, so no check needed)
					//update also the firmware version and package!
					lwm2m_store_new_value(fw_version,(void*) def_fw_version, sizeof(def_fw_version));
					lwm2m_store_new_value(fw_package,(void*) def_fw_package, sizeof(def_fw_package));
				}
			}
		}
	}
}

//----------------------------------------------------------------------------------------------
//// IAP
//#include "IAP.h"
//#define START_TARGET_SECTOR    		0
//#define END_TARGET_SECTOR    		15
//#define PAGES_PER_SECTOR			16
//#define SBL_APP_STARTING_ADDRESS	0x00000000
//#define SECTOR_SIZE					4096 //4KB sectors
//
//#define END_OF_SBL					0x0000FFFF
//#define PAGE_SIZE        			256
//#define EXT_FLASH_ID_ADDR			0x80000 //after 512K
//#define EXT_FLASH_ID_ADDR_LEN 		4
//void flash_secondary_boot_loader(void){
//	char mem[PAGE_SIZE];
//	IAP     iap;
//	N25Q * ExtFlash = NULL;
//	int sector_address, page_address, stop_address, ret_val, i, j, k;
//	int fwid[PAGE_SIZE];
//	uint8_t * ptr_page;
//
//    ExtFlash = new N25Q();
//
//	bool validextrom = false;
//	bool skip = false;
//	page_address = EXT_FLASH_ID_ADDR;
//	ExtFlash->ReadDataFromAddress(fwid, page_address, EXT_FLASH_ID_ADDR_LEN);
//	for (i=0;i<EXT_FLASH_ID_ADDR_LEN;i++){
//		if (fwid[i] != 0xFF){
//			validextrom = true;
//			break;
//		}
//	}
//	// validextrom?
//	if (validextrom){
//		stop_address = END_OF_SBL;
//		printf("------------------------ SBL ERASING!! ------------------------\r\n");
//		for (i=START_TARGET_SECTOR; i<=END_TARGET_SECTOR; i++){
//			ret_val = iap.blank_check( i, i);
//			printf("sector %d check, result = 0x%08X ", i, ret_val);
//			if (ret_val == SECTOR_NOT_BLANK){
//				printf(" not blank! ");
//				printf("... prepare and erase sector %d ... ",i);
//				// erased block
//				iap.prepare(i, i);
//				ret_val = iap.erase(i,i);
//				printf("erased!!= 0x%08X", ret_val);
//				printf("\r\n");
//			}
//			if (ret_val == CMD_SUCCESS){
//				printf(" blank!\r\n");
//			}
//		}
//		//clean, now write
//		printf("----------------------- SBL UPDATE!! --------------------------\r\n");
//		for (i=START_TARGET_SECTOR; i<=END_TARGET_SECTOR; i++){
//			if (skip == true){
//				break;
//			}
//			for (j=0;j<PAGES_PER_SECTOR;j++){
//				sector_address = SBL_APP_STARTING_ADDRESS + ((i-START_TARGET_SECTOR) * SECTOR_SIZE);
//				page_address = sector_address + (j * PAGE_SIZE);
//				printf("sector (%d | %X), page (%d | %X)\r\n",i,sector_address,j, page_address);
//				ExtFlash->ReadDataFromAddress(fwid,page_address, PAGE_SIZE);
//				ptr_page = (uint8_t *) page_address;
////    			printf("OLD DATA:\r\n");
////    			for (k=0;k<PAGE_SIZE;k++){
////    				printf("%02X ", ptr_page[k]);
////    			}
////    			printf("\r\n");
////    			printf("NEW DATA:\r\n");
//				for (k=0;k<PAGE_SIZE;k++){
//					mem[k] = (char) (fwid[k]);
////    				printf("%02X ", mem[k]);
//				}
////    			printf("\r\n");
//				// write page
//				iap.prepare(i, i);
//				printf("prepare from ram %X to flash %X \r\n", mem, page_address);
//				ret_val = iap.write  (mem, page_address,PAGE_SIZE);
//				printf("copied: Flash(0x%08X) for %d bytes. (result=0x%08X) ", page_address, PAGE_SIZE, ret_val);
//				ret_val = iap.compare(mem, page_address,PAGE_SIZE);
//				printf("| compare result = \"%s\"\r\n", ret_val ? "FAILED" : "OK" );
////    			printf("RE-WRITTEN DATA:\r\n");
////				for (k=0;k<PAGE_SIZE;k++){
////					printf("%02X ", ptr_page[k]);
////				}
////				printf("\r\n");
//				if (page_address == stop_address){
//					skip = true;
//					printf("last page written @ %X\r\n",page_address);
//					break;
//				}
//			}
//		}
//		printf("----------------------- SBL DOWNLOADED! -----------------------\r\n");
//    }
//}
