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
#include "rtos.h"
#include "n25q.h"
#include "mbed_api_wrapper.h"
#include <math.h>
#include "flash_addresses.h"
#include "IAP.h"



// mbed IAP location
#define MY_IAP_LOCATION          0x1FFF1FF1

#define SAMPLE_TIME_US			280
#define DELTA_TIME_US			40
#define DELAY_TIME_US			9680

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

// In Application Programming pointer to function
typedef void (*MyIAP) (unsigned int [], unsigned int []);

// Dust Sensor
static DigitalOut * LedDust = NULL;
static AnalogIn * DustSensor = NULL;

// Mic Sensor
static AnalogIn * MicSensor = NULL;

// Ranging Sensor
static DigitalOut * RangingSensorEnable = NULL;
static AnalogIn * RangingSensor = NULL;

// Quieting input sampling
static DigitalOut * ADCNoiseReduction = NULL;

// External Flash Memory
static N25Q * ExternalFlash = NULL;

void reboot_mbed(void){
//	fprintf(stdout, "\n\t Going down for a reboot in 5 seconds\r\n\n");
//	wait(5);
	// reboot
	NVIC_SystemReset();
}

char * get_model_number(char * buffer)
{
	// 54 command gets the NXP LPC part value
	unsigned int command[5] = {54,0,0,0,0};
	unsigned int output[5] = {12,0,0,0,0};
	MyIAP iap_entry;

	iap_entry = (MyIAP) MY_IAP_LOCATION;
	iap_entry(command, output);

	if(output[0]==0){ // CMD_SUCCESS
		if ((output[1] & 0xFFF00000) == 0x26000000){
			sprintf(buffer, "Arch-Pro");
		}
		if ((output[1] & 0xFFF00000) == 0x29800000){
			sprintf(buffer, "Arch");
		}
	}
	else{
		printf("IAP Error!\r\n");
		sprintf(buffer, "Internal Error!!");
	}
	return buffer;
}


char * get_serial_number(char * buffer)
{
	// 58 command gets the NXP LPC part serial number
	unsigned int command[5] = {58,0,0,0,0};
	unsigned int output[5] = {12,0,0,0,0};
	MyIAP iap_entry;

	iap_entry = (MyIAP) MY_IAP_LOCATION;
	iap_entry(command, output);

	if(output[0]==0){ // CMD_SUCCESS
		sprintf(buffer, "#%X-%X-%X-%X",output[1], output[2], output[3], output[4]);
	}
	else{
		printf("IAP Error!\r\n");
		sprintf(buffer, "Internal Error!!");
	}
	return buffer;
}

char * get_part_id(char * buffer)
{
	// 54 command gets the NXP LPC part id
	unsigned int command[5] = {54,0,0,0,0};
	unsigned int output[5] = {12,0,0,0,0};
	MyIAP iap_entry;

	iap_entry = (MyIAP) MY_IAP_LOCATION;
	iap_entry(command, output);

	if(output[0]==0){ // CMD_SUCCESS
		if ((output[1] & 0xFFF00000) == 0x26000000){
			sprintf(buffer, "LPC1768-%X",output[1]);
		}
		if ((output[1] & 0xFFF00000) == 0x29800000){
			sprintf(buffer, "LPC1124-%X",output[1]);
		}
	}
	else{
		printf("IAP Error!\r\n");
		sprintf(buffer, "Internal Error!!");
	}
	return buffer;
}

char * get_boot_code_version(char * buffer)
{
	// 55 command gets the Boot Code Version Number
	unsigned int command[5] = {55,0,0,0,0};
	unsigned int output[5] = {12,0,0,0,0};
	MyIAP iap_entry;

	iap_entry = (MyIAP) MY_IAP_LOCATION;
	iap_entry(command, output);

	if(output[0]==0){ // CMD_SUCCESS
		unsigned short lowvers = output[1] & 0xFF;
		unsigned short highvers = (output[1] >> 8) & 0xFF;
		sprintf(buffer, "Boot Code Vers. %hu.%hu",highvers,lowvers);
	}
	else{
		printf("IAP Error!\r\n");
		sprintf(buffer, "Internal Error!!");
		// error (check LPC documentation
	}
	return buffer;
}

char * get_mbed_version(char * buffer)
{
	sprintf(buffer, "mbed library Vers. %d",MBED_LIBRARY_VERSION);
	return buffer;
}

//NB: needs to be done at least one at beginning to initialize RTC
void mbed_set_time(long timevalue){
	set_time(timevalue);
}

//----------------------------------------------------------------------------------------------



//----------------------------------------------------------------------------------------------

// NB: working APIs
//unsigned int sample_dust_adc(void){
//	unsigned int retval;
//	LedDust->write(0);
//	wait_us(SAMPLE_TIME_US);
//	retval = DustSensor->read_u16();
//	wait_us(DELTA_TIME_US);
//	LedDust->write(1);
//	return retval;
//}

// NB: testing APIs
unsigned int sample_dust_adc(void){
	float mov_avg, sample, cnt;
	unsigned int retval;
	Timer adcwindow;

	mov_avg = 0;
	cnt = 0;
	adcwindow.start();
	do {
		LedDust->write(0);
		wait_us(SAMPLE_TIME_US);
		sample = DustSensor->read();
		wait_us(DELTA_TIME_US);
		LedDust->write(1);
		wait_us(DELAY_TIME_US);
		mov_avg = mov_avg + sample;
		cnt = cnt + 1;
	} while (adcwindow.read_ms() <= 50);
	adcwindow.stop();

	mov_avg = mov_avg / cnt;
	mov_avg = mov_avg * 65536;
	retval = (unsigned int) (mov_avg + 0.5);
	return retval;
}

void init_dust_sensor(void){
#if defined(TARGET_ARCH_PRO)
	if (LedDust == NULL){
		LedDust = new DigitalOut(P0_24);
	}
	if (DustSensor == NULL){
		DustSensor = new AnalogIn(P0_25);
	}
#endif
}

//----------------------------------------------------------------------------------------------

// NB: CURRENT API
void init_mic_sensor(void){
#if defined(TARGET_ARCH_PRO)
	if (MicSensor == NULL){
		MicSensor = new AnalogIn(P0_23);
	}
#endif
}

// NB: CURRENT API
unsigned int sample_mic_adc(void){
	float mov_avg, avg, sample, cnt;
	unsigned int retval, average;
	Timer adcwindow;

	mov_avg = 0;
	avg = 0;
	cnt = 0;
	adcwindow.start();
	do {
		sample = MicSensor->read();
		wait_us(20);
		avg = avg + sample;
		mov_avg = mov_avg + (sample * sample);
		cnt = cnt + 1;
	} while (adcwindow.read_ms() <= 50);
	adcwindow.stop();

	mov_avg = mov_avg / cnt;
	avg = avg / cnt;
	mov_avg = sqrtf(mov_avg);
	mov_avg = mov_avg * 65536; // between 65536 and 0
	avg = avg * 65536; // between 65536 and 0
	retval = (unsigned int) (mov_avg + 0.5);
	average = (unsigned int) (avg + 0.5);
	retval = retval - average;
	return retval;
}


//void background_mic_thread(void const * args){
//	float sample, sum_samples;
//	int z;
//	while(true){
////		sum_samples = 0;
//		for (z=0;z < MIC_WINDOW; z++){
//			wait_us(MIC_TIME_US);
////			sample = MicSensor->read();
//			sum_samples = MicSensor->read();
////			sum_samples = sum_samples + (sample * sample / (float) MIC_WINDOW);
//		}
////		sum_samples = sqrtf(sum_samples); //root
//		MicMutex.lock();
//		last_mic_sample = sum_samples;
//		MicMutex.unlock();
//	}
//}
//
//
//void init_mic_sensor(void){
//#if defined(TARGET_ARCH_PRO)
//	if (MicSensor == NULL){
//		MicSensor = new AnalogIn(P0_23);
//	}
//	if (MicSampling == NULL){
//		// not present, create a new thread
//		MicSampling = new Thread(background_mic_thread);
//	}
//	// so to build up a sample
//	wait_ms(50);
//#endif
//}
//
//unsigned int sample_mic_adc(void){
//	float tempval = 0;
//	unsigned int retval = 0;
//	MicMutex.lock();
//	tempval = last_mic_sample;
//	MicMutex.unlock();
//	tempval = tempval * 65536;
//	retval = (unsigned int) (tempval + 0.5);
//	return retval;
//}

//void init_mic_sensor(void){
//#if defined(TARGET_ARCH_PRO)
//	if (MicSensor == NULL){
//		MicSensor = new AnalogIn(P0_23);
//	}
//#endif
//}
//
//unsigned int sample_mic_adc(void){
//	//20 microseconds = 50khz
//	//2500 samples at 50Khz is 50ms, enough for 20Khz frequencies
//	float sum_samples = 0;
//	float sample;
//	unsigned int retval;
//	for (int i=0;i < 2500; i++){
//		wait_us(20);
//		sample = MicSensor->read();
//		sum_samples = sum_samples + (sample * sample / (float) 2500);
//	}
//	sum_samples = sqrtf(sum_samples);
//	sum_samples = sum_samples * 65536;
//	retval = (unsigned int) (sum_samples + 0.5);
//	return retval;
//}

//----------------------------------------------------------------------------------------------

void init_ranging_sensor(void){
#if defined(TARGET_ARCH_PRO)
	if (RangingSensorEnable == NULL){
		RangingSensorEnable = new DigitalOut(P1_30);
		RangingSensorEnable->write(1);
		wait_ms(26); //16.5 + 3.7 + 5 = 25.2
	}
	if (RangingSensor == NULL){
		RangingSensor = new AnalogIn(P1_31);
		// quiets down analogin
		ADCNoiseReduction = new DigitalOut(P0_26);
	}
#endif
}

unsigned int sample_ranging_adc(void){
	unsigned int retval;
	retval = RangingSensor->read_u16();
	return retval;
}

//----------------------------------------------------------------------------------------------

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

bool has_watchdog_barked(void){
#if defined(TARGET_ARCH_PRO)
	if ((LPC_WDT->WDMOD >>2) & 1){
		return true;
	}
#endif
	return false;
}

void watchdog_pet(void){
#if defined(TARGET_ARCH_PRO)
    LPC_WDT->WDFEED = 0xAA;
    LPC_WDT->WDFEED = 0x55;
#endif
}

void watchdog_kick(int deadline){
	float s = (float) deadline;
#if defined(TARGET_ARCH_PRO)
    LPC_WDT->WDCLKSEL = 0x1;                // Set CLK src to PCLK
    uint32_t clk = SystemCoreClock / 16;    // WD has a fixed /4 prescaler, PCLK default is /4
    LPC_WDT->WDTC = s * (float)clk;
    LPC_WDT->WDMOD = 0x3;                   // Enabled and Reset
#endif
    watchdog_pet();
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


