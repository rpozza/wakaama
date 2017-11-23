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
#include "ranging.h"

// Ranging Sensor
static DigitalOut * RangingSensorEnable = NULL;
static FastAnalogIn * RangingSensor = NULL;

// Quieting input sampling
static DigitalOut * ADCNoiseReduction = NULL;

void init_ranging_sensor(void){
#if defined(TARGET_ARCH_PRO)
	if (RangingSensorEnable == NULL){
		RangingSensorEnable = new DigitalOut(P1_30);
		RangingSensorEnable->write(1);
		wait_ms(26); //16.5 + 3.7 + 5 = 25.2
	}
	if (RangingSensor == NULL){
		RangingSensor = new FastAnalogIn(P1_31);
		// quiets down analogin
		ADCNoiseReduction = new DigitalOut(P0_26);
	}
#endif
}

static inline void order(unsigned short *a, unsigned short *b) {
	// a > b => swap so (a,b) -> (b,a)
    if (*a > *b) {
        unsigned short t = *a;
        *a = *b;
        *b = t;
    }
}

unsigned int sample_ranging_adc(void){
	unsigned int retval;
	unsigned short v1 = RangingSensor->read_u16();
	unsigned short v2 = RangingSensor->read_u16();
	unsigned short v3 = RangingSensor->read_u16();
	order(&v1, &v2);
	order(&v2, &v3);
	order(&v1, &v2);
	retval = v2;
	return retval;
}
