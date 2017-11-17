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

#include "buzzer.h"

// Buzzer
static Thread * BuzzerDutyCycling = NULL;
static DigitalOut * Buzzer = NULL;
static unsigned int ton_ms;
static unsigned int toff_ms;

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
