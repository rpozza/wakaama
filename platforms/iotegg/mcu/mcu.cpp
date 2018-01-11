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
#include "mcu.h"

// mbed IAP location
#define MY_IAP_LOCATION          0x1FFF1FF1

// In Application Programming pointer to function
typedef void (*MyIAP) (unsigned int [], unsigned int []);

void reboot_mcu(void){
//	wait(5);
	// reboot
	wait_ms(100);
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
		// error (check LPC documentation)
	}
	return buffer;
}

char * get_mbed_version(char * buffer)
{
	sprintf(buffer, "mbed library Vers. %d",MBED_LIBRARY_VERSION);
	return buffer;
}

//NB: needs to be done at least one at beginning to initialize RTC
void mcu_set_time(long timevalue){
	set_time(timevalue);
}

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
