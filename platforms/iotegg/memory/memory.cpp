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
#include "flash_addresses.h"
#include "IAP.h"

#include "memory.h"

// IAP
#define START_TARGET_SECTOR    		0
#define END_TARGET_SECTOR    		15
#define PAGES_PER_SECTOR			16
#define SBL_APP_STARTING_ADDRESS	0x00000000
#define SECTOR_SIZE					4096 //4KB sectors

#define END_OF_SBL					0x0000FFFF
#define PAGE_SIZE        			256
#define EXT_FLASH_ID_ADDR			0x80000 //after 512K
#define EXT_FLASH_ID_ADDR_LEN 		4

// External Flash Memory
static N25Q * ExternalFlash = NULL;

void init_ext_flash(void){
#if defined(TARGET_ARCH_PRO)
	if (ExternalFlash == NULL){
		ExternalFlash = new N25Q();
	}
#endif
}

void erase_ext_flash(void){
	ExternalFlash->NonBlockingBulkErase();
}

bool flash_is_busy(void){
	return ExternalFlash->isBusy();
}

void flash_program_page(uint8_t * data, uint32_t address, int length){
	int tempArray[PAGE_SIZE] = {0};
	int tempAddress = (int) address;
	for (int i = 0; i< length; i++){
		tempArray[i] = (int) data[i];
	}
	ExternalFlash->NonBlockingProgramFromAddress(tempArray, tempAddress, length);
}

void flash_read_page(uint8_t * data, uint32_t address, int length){
	int tempArray[PAGE_SIZE] = {0};
	int tempAddress = (int) address;
	ExternalFlash->ReadDataFromAddress(tempArray, tempAddress, length);
	for (int i = 0; i< length; i++){
		data[i] = (uint8_t) tempArray[i];
	}
}

void flash_write_blocking(uint8_t * data, uint32_t address, int length){
	int tempArray[PAGE_SIZE] = {0};
	int tempAddress = (int) address;
	for (int i = 0; i< length; i++){
		tempArray[i] = (int) data[i];
	}
	ExternalFlash->ProgramFromAddress(tempArray, tempAddress, length);
}

void flash_read_blocking(uint8_t * data, uint32_t address, int length){
	int tempArray[PAGE_SIZE] = {0};
	int tempAddress = (int) address;
	ExternalFlash->ReadDataFromAddress(tempArray, tempAddress, length);
	for (int i = 0; i< length; i++){
		data[i] = (uint8_t) tempArray[i];
	}
}

bool is_default(uint32_t address){
	uint8_t flag[1];
	flash_read_blocking(flag,address,1);
	if (flag[0]==0xFF){
		return true;
	}
	return false;
}

void make_dirt(uint32_t address){
	uint8_t flag[1] = {0};
	flash_write_blocking(flag,address,1);
}

void serialize_uint32_t (uint32_t value, uint32_t address){
	uint8_t buffer[4];//max len of variables

	buffer[0] = value & 0xFF;
	buffer[1] = (value >> 8) & 0xFF;
	buffer[2] = (value >> 16) & 0xFF;
	buffer[3] = (value >> 24) & 0xFF;

	flash_subsector_4K_erase(address);
	flash_write_blocking(buffer,address, 4);
}

uint32_t deserialize_uint32_t (uint32_t address){
	uint32_t ret_val = 0;
	uint8_t buffer[4];

	flash_read_blocking(buffer,address, 4);

	ret_val = buffer[3] << 24 | buffer[2] << 16 | buffer[1] << 8 | buffer[0];
	return ret_val;
}

void serialize_char_t (char * value, int len, uint32_t address){

	flash_subsector_4K_erase(address);
	flash_write_blocking((uint8_t *) value,address, len);
}

void deserialize_char_t (char * value, uint32_t address, int len){
	flash_read_blocking((uint8_t *) value,address, len);
}

void serialize_bool_t (bool value, uint32_t address){
	uint8_t buffer[1];

	buffer[0] = (uint8_t) value;
	flash_subsector_4K_erase(address);
	flash_write_blocking(buffer,address,1);
}

bool deserialize_bool_t (uint32_t address){
	uint8_t buffer[1];
	flash_read_blocking(buffer,address,1);
	return (bool) buffer[0];
}

void serialize_uint8_t (uint8_t value, uint32_t address){
	uint8_t buffer[1];

	buffer[0] = value;
	flash_subsector_4K_erase(address);
	flash_write_blocking(buffer,address,1);
}

uint8_t deserialize_uint8_t (uint32_t address){
	uint8_t buffer[1];
	flash_read_blocking(buffer,address,1);
	return buffer[0];
}

void default_ext_flash_values(void){
	char doublebuf[sizeof(double)] = {0};
	double var;
	if (is_default(OBJ_1_RES_1_ADDR+OBJ_FLAG_START)){
		serialize_uint32_t(PRV_DEF_LIFETIME,OBJ_1_RES_1_ADDR);
		make_dirt(OBJ_1_RES_1_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_1_RES_2_ADDR+OBJ_FLAG_START)){
		serialize_uint32_t(PRV_DEF_MIN_PERIOD,OBJ_1_RES_2_ADDR);
		make_dirt(OBJ_1_RES_2_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_1_RES_3_ADDR+OBJ_FLAG_START)){
		serialize_uint32_t(PRV_DEF_MAX_PERIOD,OBJ_1_RES_3_ADDR);
		make_dirt(OBJ_1_RES_3_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_3_RES_14_ADDR+OBJ_FLAG_START)){
		serialize_char_t(PRV_UTC_OFFSET,PRV_OFFSET_MAXLEN,OBJ_3_RES_14_ADDR);
		make_dirt(OBJ_3_RES_14_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_3311_RES_5706_ADDR+OBJ_FLAG_START)){
		serialize_char_t(PRV_DEF_COLOUR, PRV_DEF_COLOUR_LEN, OBJ_3311_RES_5706_ADDR);
		make_dirt(OBJ_3311_RES_5706_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_3311_RES_5850_ADDR+OBJ_FLAG_START)){
		serialize_bool_t(PRV_3311_DEF_STATE, OBJ_3311_RES_5850_ADDR);
		make_dirt(OBJ_3311_RES_5850_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_3311_RES_5851_ADDR+OBJ_FLAG_START)){
		serialize_uint8_t(PRV_3311_DEF_DIMMER, OBJ_3311_RES_5851_ADDR);
		make_dirt(OBJ_3311_RES_5851_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_3324_RES_5821_ADDR+OBJ_FLAG_START)){
		var = PRV_3324_CALIBRATION;
		memcpy(doublebuf,&var,sizeof(double));
		serialize_char_t(doublebuf,sizeof(double), OBJ_3324_RES_5821_ADDR);
		make_dirt(OBJ_3324_RES_5821_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_3324_RES_5750_ADDR+OBJ_FLAG_START)){
		serialize_char_t(PRV_3324_APPLICATION,APPLICATION_BUFFER_LEN,OBJ_3324_RES_5750_ADDR);
		make_dirt(OBJ_3324_RES_5750_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_3325_RES_5821_ADDR+OBJ_FLAG_START)){
		var = PRV_3325_CALIBRATION;
		memcpy(doublebuf,&var,sizeof(double));
		serialize_char_t(doublebuf,sizeof(double), OBJ_3325_RES_5821_ADDR);
		make_dirt(OBJ_3325_RES_5821_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_3325_RES_5750_ADDR+OBJ_FLAG_START)){
		serialize_char_t(PRV_3325_APPLICATION,APPLICATION_BUFFER_LEN,OBJ_3325_RES_5750_ADDR);
		make_dirt(OBJ_3325_RES_5750_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_3330_RES_5821_ADDR+OBJ_FLAG_START)){
		var = PRV_3330_CALIBRATION;
		memcpy(doublebuf,&var,sizeof(double));
		serialize_char_t(doublebuf,sizeof(double), OBJ_3330_RES_5821_ADDR);
		make_dirt(OBJ_3330_RES_5821_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_3330_RES_5750_ADDR+OBJ_FLAG_START)){
		serialize_char_t(PRV_3330_APPLICATION,APPLICATION_BUFFER_LEN,OBJ_3330_RES_5750_ADDR);
		make_dirt(OBJ_3330_RES_5750_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_3338_RES_5850_ADDR+OBJ_FLAG_START)){
		serialize_bool_t(PRV_3338_DEF_STATE, OBJ_3338_RES_5850_ADDR);
		make_dirt(OBJ_3338_RES_5850_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_3338_RES_5851_ADDR+OBJ_FLAG_START)){
		serialize_uint8_t(PRV_3338_DEF_DIMMER, OBJ_3338_RES_5851_ADDR);
		make_dirt(OBJ_3338_RES_5851_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_3338_RES_5521_ADDR+OBJ_FLAG_START)){
		var = PRV_3338_DURATION;
		memcpy(doublebuf,&var,sizeof(double));
		serialize_char_t(doublebuf,sizeof(double), OBJ_3338_RES_5521_ADDR);
		make_dirt(OBJ_3338_RES_5521_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_3338_RES_5525_ADDR+OBJ_FLAG_START)){
		var = PRV_3338_MIN_TIME_OFF;
		memcpy(doublebuf,&var,sizeof(double));
		serialize_char_t(doublebuf,sizeof(double), OBJ_3338_RES_5525_ADDR);
		make_dirt(OBJ_3338_RES_5525_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_3338_RES_5750_ADDR+OBJ_FLAG_START)){
		serialize_char_t(PRV_3338_APPLICATION,APPLICATION_BUFFER_LEN,OBJ_3338_RES_5750_ADDR);
		make_dirt(OBJ_3338_RES_5750_ADDR+OBJ_FLAG_START);
	}
	if (is_default(OBJ_3348_RES_5750_ADDR+OBJ_FLAG_START)){
		serialize_char_t(PRV_3348_APPLICATION,APPLICATION_BUFFER_LEN,OBJ_3348_RES_5750_ADDR);
		make_dirt(OBJ_3348_RES_5750_ADDR+OBJ_FLAG_START);
	}
}

void flash_subsector_4K_erase(uint32_t address){
	int tempAddress = (int) address;
	ExternalFlash->SubSectorErase(tempAddress);
}

void flash_sector_64K_erase(uint32_t address){
	int tempAddress = (int) address;
	ExternalFlash->SectorErase(tempAddress);
}

//----------------------------------------------------------------------------------------------

void flash_secondary_boot_loader(void){
	char mem[PAGE_SIZE];
	IAP     iap;
	N25Q * ExtFlash = NULL;
	int sector_address, page_address, stop_address, ret_val, i, j, k;
	int fwid[PAGE_SIZE];
	uint8_t * ptr_page;

    ExtFlash = new N25Q();

	bool validextrom = false;
	bool skip = false;
	page_address = EXT_FLASH_ID_ADDR;
	ExtFlash->ReadDataFromAddress(fwid, page_address, EXT_FLASH_ID_ADDR_LEN);
	for (i=0;i<EXT_FLASH_ID_ADDR_LEN;i++){
		if (fwid[i] != 0xFF){
			validextrom = true;
			break;
		}
	}
	// validextrom?
	if (validextrom){
		stop_address = END_OF_SBL;
		printf("------------------------ SBL ERASING!! ------------------------\r\n");
		for (i=START_TARGET_SECTOR; i<=END_TARGET_SECTOR; i++){
			ret_val = iap.blank_check( i, i);
			printf("sector %d check, result = 0x%08X ", i, ret_val);
			if (ret_val == SECTOR_NOT_BLANK){
				printf(" not blank! ");
				printf("... prepare and erase sector %d ... ",i);
				// erased block
				iap.prepare(i, i);
				ret_val = iap.erase(i,i);
				printf("erased!!= 0x%08X", ret_val);
				printf("\r\n");
			}
			if (ret_val == CMD_SUCCESS){
				printf(" blank!\r\n");
			}
		}
		//clean, now write
		printf("----------------------- SBL UPDATE!! --------------------------\r\n");
		for (i=START_TARGET_SECTOR; i<=END_TARGET_SECTOR; i++){
			if (skip == true){
				break;
			}
			for (j=0;j<PAGES_PER_SECTOR;j++){
				sector_address = SBL_APP_STARTING_ADDRESS + ((i-START_TARGET_SECTOR) * SECTOR_SIZE);
				page_address = sector_address + (j * PAGE_SIZE);
				printf("sector (%d | %X), page (%d | %X)\r\n",i,sector_address,j, page_address);
				ExtFlash->ReadDataFromAddress(fwid,page_address, PAGE_SIZE);
				ptr_page = (uint8_t *) page_address;
//    			printf("OLD DATA:\r\n");
//    			for (k=0;k<PAGE_SIZE;k++){
//    				printf("%02X ", ptr_page[k]);
//    			}
//    			printf("\r\n");
//    			printf("NEW DATA:\r\n");
				for (k=0;k<PAGE_SIZE;k++){
					mem[k] = (char) (fwid[k]);
//    				printf("%02X ", mem[k]);
				}
//    			printf("\r\n");
				// write page
				iap.prepare(i, i);
				printf("prepare from ram %X to flash %X \r\n", mem, page_address);
				ret_val = iap.write  (mem, page_address,PAGE_SIZE);
				printf("copied: Flash(0x%08X) for %d bytes. (result=0x%08X) ", page_address, PAGE_SIZE, ret_val);
				ret_val = iap.compare(mem, page_address,PAGE_SIZE);
				printf("| compare result = \"%s\"\r\n", ret_val ? "FAILED" : "OK" );
//    			printf("RE-WRITTEN DATA:\r\n");
//				for (k=0;k<PAGE_SIZE;k++){
//					printf("%02X ", ptr_page[k]);
//				}
//				printf("\r\n");
				if (page_address == stop_address){
					skip = true;
					printf("last page written @ %X\r\n",page_address);
					break;
				}
			}
		}
		printf("----------------------- SBL DOWNLOADED! -----------------------\r\n");
    }
}
