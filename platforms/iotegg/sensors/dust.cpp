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
#include "dust.h"

/* Dust and Smoke Sensor: GP2Y1010AU0F
 * Pulse of 10ms+-1ms, with pulse width of 0.32ms +- 0.02ms
 * Led is turned on with GND, after 0.28ms, sampling needs to take place and then hold on
 * Led is turned off with VDD at 0.32ms, then cycled with 10ms interval
 */

#define ON_TIME_US				280
#define ON_DELTA_US				40
#define PERIOD_TIME_US			9680
#define NUMBER_CYCLES			5

// Dust Sensor
static DigitalOut * LedDust = NULL;
static FastAnalogIn * DustSensor = NULL;

unsigned int sample_dust_adc(void){
	float curr_avg, prev_avg, sample, t;
	unsigned int retval;

	curr_avg = 0;
	prev_avg = 0;
	for (t=0; t < NUMBER_CYCLES; t++){
		LedDust->write(0); // LED turn-on
		wait_us(ON_TIME_US);
		sample = DustSensor->read();
		wait_us(ON_DELTA_US);
		LedDust->write(1);
		wait_us(PERIOD_TIME_US);
		curr_avg = prev_avg + (1.0f / ((float) t + 1)) * (sample - prev_avg);
		prev_avg = curr_avg;
	}
	retval = (unsigned int) (curr_avg * (65535.0f/1.0f));
	return retval;
}

void init_dust_sensor(void){
#if defined(TARGET_ARCH_PRO)
	if (LedDust == NULL){
		LedDust = new DigitalOut(P0_24);
	}
	if (DustSensor == NULL){
		DustSensor = new FastAnalogIn(P0_25);
	}
#endif
}
