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
#include "HTU21D.h"
#include "temp_humd.h"

// TEMPERATURE & HUMIDITY SENSOR
static HTU21D * THSensor = NULL;

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
