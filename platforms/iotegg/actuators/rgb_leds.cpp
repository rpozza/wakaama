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
#include "rgb_leds.h"

// LEDs
static PwmOut * Lred = NULL;
static PwmOut * Lgreen = NULL;
static PwmOut * Lblue = NULL;

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
