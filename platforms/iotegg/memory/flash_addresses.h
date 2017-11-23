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

#ifndef FLASH_ADDRESSES_H_
#define FLASH_ADDRESSES_H_

#define OBJ_FLAG_START				0x20000 // 4KB page

#define OBJ_1_RES_1_ADDR			0xA0000
#define PRV_DEF_LIFETIME			300
#define OBJ_1_RES_2_ADDR			0xA1000
#define PRV_DEF_MIN_PERIOD  		0
#define OBJ_1_RES_3_ADDR			0xA2000
#define PRV_DEF_MAX_PERIOD  		1

#define OBJ_3_RES_14_ADDR			0xA3000
#define PRV_UTC_OFFSET		  		"+00:00"
#define PRV_OFFSET_MAXLEN   		7 //+HH:MM\0 at max

#define OBJ_3311_RES_5706_ADDR		0xA4000
#define PRV_DEF_COLOUR		  		"#000000"
#define PRV_DEF_COLOUR_LEN			8
#define OBJ_3311_RES_5850_ADDR		0xA5000
#define PRV_3311_DEF_STATE		  	false
#define OBJ_3311_RES_5851_ADDR		0xA6000
#define PRV_3311_DEF_DIMMER			100

#define OBJ_3324_RES_5821_ADDR		0xA7000
#define PRV_3324_CALIBRATION		0			  // dBV output
#define OBJ_3324_RES_5750_ADDR		0xA8000
#define PRV_3324_APPLICATION		"Loudness Sensor"
#define APPLICATION_BUFFER_LEN		20

#define OBJ_3325_RES_5821_ADDR		0xA9000
#define PRV_3325_CALIBRATION        (0.5 / 2.8)
#define OBJ_3325_RES_5750_ADDR		0xAA000
#define PRV_3325_APPLICATION		"Dust Sensor"


#define OBJ_3330_RES_5821_ADDR		0xAB000
#define PRV_3330_CALIBRATION        0
#define OBJ_3330_RES_5750_ADDR		0xAC000
#define PRV_3330_APPLICATION 		"Ranging Sensor"

#define OBJ_3338_RES_5521_ADDR		0xAD000
#define PRV_3338_DURATION			0
#define OBJ_3338_RES_5525_ADDR		0xAE000
#define PRV_3338_MIN_TIME_OFF		0
#define OBJ_3338_RES_5750_ADDR		0xAF000
#define PRV_3338_APPLICATION		"Buzzer/Alarm"
#define OBJ_3338_RES_5850_ADDR		0xB0000
#define	PRV_3338_DEF_STATE			false
#define OBJ_3338_RES_5851_ADDR		0xB1000
#define PRV_3338_DEF_DIMMER			100

#define OBJ_3348_RES_5750_ADDR		0xB2000
#define PRV_3348_APPLICATION		"Gesture Sensor"

#endif
