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
#include "SparkFun_APDS9960.h"
#include "gesture_light.h"

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

// Interrupt Service Routine, setting a flag
void gesture_isr_routine(void){
	gesture_isr_flag = true;
}

// Thread reading a Gesture value
void gesture_thread(void const * args){
	GestureMutex.lock();
	if (GestureSensor->isGestureAvailable()){
		last_gesture_sample = GestureSensor->readGesture();
	}
	GestureMutex.unlock();
	enable_gesture_irq();
}

// Initialization
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

// Freeing resources
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

// Enable/Disable Interrupts
void disable_gesture_irq(void){
	GestureTrigger->disable_irq();
}

void enable_gesture_irq(void){
	GestureTrigger->enable_irq();
}

// Service Routine Flags
bool is_gesture_isr_flag_set(void){
	return gesture_isr_flag;
}

void reset_gesture_isr_flag(void){
	gesture_isr_flag = false;
}

// Interrupt Handler (manually called on flag set)
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

// Return last value
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
