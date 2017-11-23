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
#include "FastAnalogIn.h"
#include "audio.h"
#include <math.h>

#define SAMPLE_PERIOD_US 		 50 			//fs = 20KHz
#define DC_OFFSET				 0.5			//half Vcc

// Mic Sensor
static FastAnalogIn * MicSensor = NULL;
static Ticker * MicTicker = NULL;

static unsigned int t = 0;
static float s = 0;

void mic_handler(void){ //called at every sampling interval
	float x,a;
	x = MicSensor->read();  //read output amplitude
	a = fabs(x-DC_OFFSET);  //remove offset
	s += a*a;				//square
	t++;
}

void init_mic_sensor(void){
#if defined(TARGET_ARCH_PRO)
	if (MicSensor == NULL){
		MicSensor = new FastAnalogIn(P0_23);
		wait_ms(1); //let it run a while
	}
#endif
	if (MicTicker == NULL){
		MicTicker = new Ticker();
	}
	t = 0;
	s = 0;
	MicTicker->attach_us(&mic_handler,SAMPLE_PERIOD_US);
}

unsigned int get_last_rms(void){
	unsigned int retval = 0;
	float mic_rms;
	MicTicker->detach(); //remove interrupt sampling
	s = s / t;
	mic_rms = sqrtf(s);  //root of mean of squares
	retval = (unsigned int) (mic_rms * (65535.0f/1.0f));
//	printf("ViRMS: %f",mic_rms);
	// cleanup and return
	t = 0;
	s = 0;
	MicTicker->attach_us(&mic_handler,SAMPLE_PERIOD_US); //reattach interrupt sampling
	return retval;
}
