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
#include "mbed_api_wrapper.h"
#include <math.h>

#define SAMPLE_TIME_US			280
#define DELTA_TIME_US			40
#define DELAY_TIME_US			9680

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

