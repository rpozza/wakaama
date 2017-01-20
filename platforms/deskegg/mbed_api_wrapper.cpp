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
#include "HTU21D.h"
#include "SparkFun_APDS9960.h"
#include "n25q.h"
#include "mbed_api_wrapper.h"
#include <math.h>

// mbed IAP location
#define IAP_LOCATION          0x1FFF1FF1

#define SAMPLE_TIME_US			280
#define DELTA_TIME_US			40
#define DELAY_TIME_US			9680

// In Application Programming pointer to function
typedef void (*IAP) (unsigned int [], unsigned int []);

// LEDs
static PwmOut * Lred = NULL;
static PwmOut * Lgreen = NULL;
static PwmOut * Lblue = NULL;

// Buzzer
static Thread * BuzzerDutyCycling = NULL;
static DigitalOut * Buzzer = NULL;
static unsigned int ton_ms;
static unsigned int toff_ms;

// Temperature and Humidity Sensor
static HTU21D * THSensor = NULL;

// Gesture Sensor
static SparkFun_APDS9960 * GestureSensor = NULL;
static I2C * GestureComms = NULL;
static InterruptIn * GestureTrigger = NULL;
static bool gesture_isr_flag = false;
static int last_gesture_sample = 0;
static Thread * GestureSampling = NULL;

// Ambient Light Sensor
static int last_ambient_sample = 0;
static Mutex GestureMutex;

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
	IAP iap_entry;

	iap_entry = (IAP) IAP_LOCATION;
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
	IAP iap_entry;

	iap_entry = (IAP) IAP_LOCATION;
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
	IAP iap_entry;

	iap_entry = (IAP) IAP_LOCATION;
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
	IAP iap_entry;

	iap_entry = (IAP) IAP_LOCATION;
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

void mbed_set_time(long timevalue){
	set_time(timevalue);
}

//----------------------------------------------------------------------------------------------

void init_rgb_leds(void){
#if defined(TARGET_ARCH_PRO)
	if (Lred == NULL){
		Lred = new PwmOut(P2_5);
	}
	if (Lgreen == NULL){
		Lgreen = new PwmOut(P2_4);
	}
	if (Lblue == NULL){
		Lblue = new PwmOut(P2_3);
	}
#endif
}

void set_red(int rgbvalue, int dimming){
	float outputvalue = 1.0f;
	outputvalue *= rgbvalue;
	outputvalue /= 255;
	outputvalue *= dimming;
	outputvalue /= 100;
//	printf("Red=%f\r\n",outputvalue);
	Lred->write(outputvalue);
}
void set_green(int rgbvalue, int dimming){
	float outputvalue = 1.0f;
	outputvalue *= rgbvalue;
	outputvalue /= 255;
	outputvalue *= dimming;
	outputvalue /= 100;
//	printf("Green=%f\r\n",outputvalue);
	Lgreen->write(outputvalue);
}
void set_blue(int rgbvalue, int dimming){
	float outputvalue = 1.0f;
	outputvalue *= rgbvalue;
	outputvalue /= 255;
	outputvalue *= dimming;
	outputvalue /= 100;
//	printf("Blue=%f\r\n",outputvalue);
	Lblue->write(outputvalue);
}

//--------------------------void init_ext_flash(void);--------------------------------------------------------------------

void init_buzzer(void){
#if defined(TARGET_ARCH_PRO)
	if (Buzzer == NULL){
		Buzzer = new DigitalOut(P2_12);
	}
#endif
}

void set_buzzer_on(void){
	Buzzer->write(1);
}

void set_buzzer_off(void){
	Buzzer->write(0);
}

void buzzer_thread(void const * args){
	set_buzzer_on();
//	printf("ton=%d, toff=%d, dim=%d\r\n",ton_ms, toff_ms, dim);
	while(true){
		if (ton_ms >0 ){
			BuzzerDutyCycling->wait(ton_ms); //ms, approx
			if (toff_ms == 0){
				// just once
				set_buzzer_off();
			}
			else if (toff_ms > 0){
				set_buzzer_off();
				BuzzerDutyCycling->wait(toff_ms); //ms, approx
				set_buzzer_on();
			}
		}
		else{
			// leave it on
		}
	}
}

void attach_buzzer_on(unsigned int on_time, unsigned int off_time){
	// store variables for thread
	ton_ms = on_time;
	toff_ms = off_time;
	//starts thread
	if (BuzzerDutyCycling == NULL){
		// not present, create a new thread
//		printf("Starting thread!\r\n");
		BuzzerDutyCycling = new Thread(buzzer_thread);
	}
}
void detach_buzzer(void){
	//stops thread
	if (BuzzerDutyCycling != NULL){
//		printf("Terminating thread!\r\n");
		BuzzerDutyCycling->terminate();
//		printf("Deleting thread!\r\n");
		delete BuzzerDutyCycling;
		BuzzerDutyCycling = NULL;
	}
	set_buzzer_off();
}

//----------------------------------------------------------------------------------------------

void init_temp_humd(void){
#if defined(TARGET_ARCH_PRO)
	if (THSensor == NULL){
		THSensor = new HTU21D(P0_27,P0_28);
	}
#endif
}

unsigned int get_raw_temperature_cel(void){
	unsigned int retval = 0;
	if (THSensor != NULL){
		retval = THSensor->sample_ctemp();
	}
	return retval;
}

unsigned int get_raw_humidity(void){
	unsigned int retval = 0;
	if (THSensor != NULL){
		retval = THSensor->sample_humid();
	}
	return retval;
}

//----------------------------------------------------------------------------------------------

void gesture_isr_routine(void){
	gesture_isr_flag = true;
}

void gesture_thread(void const * args){
	GestureMutex.lock();
	if (GestureSensor->isGestureAvailable()){
		last_gesture_sample = GestureSensor->readGesture();
	}
	GestureMutex.unlock();
	enable_gesture_irq();
}

bool init_gesture_sensor(void){
#if defined(TARGET_ARCH_PRO)
	if (GestureComms == NULL){
		GestureComms = new I2C(P0_27,P0_28);
	}
	if (GestureSensor == NULL){
		GestureSensor = new SparkFun_APDS9960(*GestureComms);
	}
	if (GestureTrigger == NULL){
		GestureTrigger = new InterruptIn(P2_13);
	}
#endif
	GestureTrigger->mode(PullUp);
	GestureTrigger->fall(&gesture_isr_routine);
	GestureTrigger->disable_irq(); // maybe this goes before

	if (!GestureSensor->init(400000)){
		printf("Gesture Sensor Init Failed!\r\n");
		return false;
	}
	if (!GestureSensor->enableGestureSensor(true)){
		printf("Gesture Sensor Enable Failed!\r\n");
		return false;
	}
	if (!GestureSensor->enableLightSensor(false)){
		printf("Ambient Sensor Enable Failed!\r\n");
		return false;
	}
	return true;
}

void free_gesture_sensor(void){
	GestureTrigger->disable_irq();
	GestureSensor->disableGestureSensor();
	GestureSensor->disableLightSensor();
	delete GestureSensor;
	GestureSensor = NULL;
	delete GestureTrigger;
	GestureTrigger = NULL;
	delete GestureComms;
	GestureComms = NULL;
}

void disable_gesture_irq(void){
	GestureTrigger->disable_irq();
}

void enable_gesture_irq(void){
	GestureTrigger->enable_irq();
}

bool is_gesture_isr_flag_set(void){
	return gesture_isr_flag;
}

void reset_gesture_isr_flag(void){
	gesture_isr_flag = false;
}

void gesture_handler(void){
	if (GestureSampling == NULL){
		GestureSampling = new Thread(gesture_thread);
	}
	else if (GestureSampling != NULL){
		GestureSampling->terminate();
		delete GestureSampling;
		GestureSampling = new Thread(gesture_thread);
	}
}

int get_last_gesture(void){
	return last_gesture_sample;
}

//----------------------------------------------------------------------------------------------
//NB: sensor shared with gesture and initialized/freed in gesture routines

int get_last_ambient_light(void){
	unsigned short int lightvalue = 0;
	if(GestureMutex.lock(10) == osOK){ // tries to lock for 10ms if returns ok , update new value
		GestureSensor->readAmbientLight(lightvalue);
		last_ambient_sample = (int) lightvalue;
		GestureMutex.unlock();
	}
	return last_ambient_sample;
}

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

void init_mic_sensor(void){
#if defined(TARGET_ARCH_PRO)
	if (MicSensor == NULL){
		MicSensor = new AnalogIn(P0_23);
	}
#endif
}

// NB previous API
//unsigned int sample_mic_adc(void){
//	unsigned int sample;
////	unsigned int peaktopeak, minval, maxval;
////	Timer adcwindow;
////
////	maxval = 0x0;
////	minval = 0xFFFF;
////	adcwindow.start();
////	do {
////		sample = MicSensor->read_u16();
////		if (sample > maxval){
////			maxval = sample;
////		}
////		if (sample < minval){
////			minval = sample;
////		}
////	} while (adcwindow.read_ms() <= 50);
////	adcwindow.stop();
////	peaktopeak = maxval - minval;
////	return peaktopeak;
//	sample = MicSensor->read_u16();
//	return sample;
//}

//// NB: previous API 2
//unsigned int sample_mic_adc(void){
//	float mov_avg, sample, cnt;
//	unsigned int retval;
//	Timer adcwindow;
//
//	mov_avg = 0;
//	cnt = 0;
//	adcwindow.start();
//	do {
//		sample = MicSensor->read();
//		wait_us(20);
//		mov_avg = mov_avg + (sample * sample);
//		cnt = cnt + 1;
//	} while (adcwindow.read_ms() <= 50);
//	adcwindow.stop();
//
//	mov_avg = mov_avg / cnt;
//	mov_avg = sqrtf(mov_avg);
//	mov_avg = mov_avg * 65536; // between 65536 and 0
//	retval = (unsigned int) (mov_avg + 0.5);
//	return retval;
//}

// NB: testing API
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

