/*******************************************************************************
 *
 * Copyright (c) 2013, 2014 Intel Corporation and others.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * The Eclipse Distribution License is available at
 *    http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    David Navarro, Intel Corporation - initial API and implementation
 *    domedambrosio - Please refer to git log
 *    Fabien Fleutot - Please refer to git log
 *    Axel Lorente - Please refer to git log
 *    Achim Kraus, Bosch Software Innovations GmbH - Please refer to git log
 *    Pascal Rieux - Please refer to git log
 *
 *******************************************************************************/

/*
 Copyright (c) 2013, 2014 Intel Corporation

 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
     * Neither the name of Intel Corporation nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 THE POSSIBILITY OF SUCH DAMAGE.

 David Navarro <david.navarro@intel.com>

*/

/*
 * Customized by Riccardo Pozza <r.pozza@surrey.ac.uk>
 * Implements a light control object according to IPSO Object 3311
 *
 *                          Multiple
 *     Object      |  ID  | Instances | Mandatory |
 *  Light Control  | 3311 |    Yes    |    No     |
 *
 *  Resources:
 *                     Supported    Multiple
 *   Name    |   ID  | Operations | Instances | Mandatory |  Type   |   Range   | Units | Description |
 *  On/Off   |  5850 |    R/W     |    No     |    Yes    | Boolean |   (0;1)   |       |             |
 *  Dimmer   |  5851 |    R/W     |    No     |    No     | Integer | (0<->100) |   %   |             |
 *  On Time  |  5852 |    R/W     |    No     |    No     | Integer |           |  sec  |             |
 *  Colour   |  5706 |    R/W     |    No     |    No     | String  |           |       |             |
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "lwm2mclient.h"
#include "liblwm2m.h"

#define BUFFER_LEN 						8

#define LIGHT_CONTROL_OBJECT_ID 		3311
#define PRV_SENS_UNIT					"RGB LED Actuator"
#define PRV_POWER_FACTOR		 		1
#define PRV_LIGHT_V				 		2	 // 2V typical LED
#define PRV_LIGHT_A				 		0.02 // 20mA typical LED

#define RES_M_ON_OFF                    5850
#define RES_M_DIMMER	                5851
#define RES_M_ON_TIME                   5852
#define RES_M_COLOUR                    5706

#define RES_M_SENSOR_UNITS				5701
#define RES_M_CUM_ACTIVE_POWER			5805
#define RES_M_POWER_FACTOR				5820


typedef struct _light_ctrl_instance_
{
	struct _light_ctrl_instance_ * next;
	uint16_t instanceId;

    bool is_on;
    uint8_t colour_red;
    uint8_t colour_green;
    uint8_t colour_blue;
    int64_t timeon;
    uint8_t dimmer;
    double cum_active_power;
    int64_t lastTimeChanged;
    double currentPowerConsumption;
    char colourCode[BUFFER_LEN];

} light_ctrl_instance_t;

static int prv_check_valid_colour(char * buffer,
                                  int length)
{
	int i;
	if (length != 7) return 0;
	if (buffer[0] != '#') return 0;
	for (i=1;i<length;i++){
		if (buffer[i] < '0' || (buffer[i] > '9' && buffer[i] < 'A') || (buffer[i] > 'F' && buffer[i] < 'a') || buffer[i] > 'f'){
			printf("Error!, Invalid string at %d = %c\r\n", i, buffer[i]);
			return 0;
		}
	}
	return 1;
}

static uint32_t prv_store_colour_string(char *bufferInput,
										uint8_t *redValue,
										uint8_t *greenValue,
										uint8_t *blueValue)
{
	char bufferCopied[BUFFER_LEN];
	uint32_t returnval = 0;

	strcpy(bufferCopied,bufferInput); // prevents dirty buffer
	returnval = (uint32_t) strtol((bufferCopied+1), (bufferCopied+7), 16);

	(*redValue) = ((returnval & 0x00FF0000) >> 16);
	(*greenValue) = ((returnval & 0x0000FF00) >> 8);
	(*blueValue) = (returnval & 0x000000FF);

	return returnval;
}

static uint8_t prv_set_value(lwm2m_data_t * dataP,
                             light_ctrl_instance_t * targetP)
{
    // a simple switch structure is used to respond at the specified resource asked
    switch (dataP->id)
    {
    case RES_M_ON_OFF:
        lwm2m_data_encode_bool(targetP->is_on, dataP);
        return COAP_205_CONTENT;

    case RES_M_DIMMER:
        lwm2m_data_encode_int(targetP->dimmer, dataP);
        return COAP_205_CONTENT;

    case RES_M_ON_TIME:
    	{
    		if (targetP->is_on){
				lwm2m_data_encode_int(time(NULL) - targetP->timeon, dataP);
			}
			else if (!targetP->is_on){
				lwm2m_data_encode_int(0, dataP);
			}
    		return COAP_205_CONTENT;
		}

    case RES_M_COLOUR:
        lwm2m_data_encode_string(targetP->colourCode, dataP);
        return COAP_205_CONTENT;

    case RES_M_SENSOR_UNITS:
    	lwm2m_data_encode_string(PRV_SENS_UNIT, dataP);
        return COAP_205_CONTENT;

    case RES_M_CUM_ACTIVE_POWER:
    	lwm2m_data_encode_float(targetP->cum_active_power + ((double)(time(NULL) - targetP->lastTimeChanged) * targetP->currentPowerConsumption), dataP);
    	return COAP_205_CONTENT;

    case RES_M_POWER_FACTOR:
    	lwm2m_data_encode_float(PRV_POWER_FACTOR, dataP);
    	return COAP_205_CONTENT;

    default:
        return COAP_404_NOT_FOUND;
    }
}

static uint8_t prv_lightctrl_read(uint16_t instanceId,
                        	 	  int * numDataP,
								  lwm2m_data_t ** dataArrayP,
								  lwm2m_object_t * objectP)
{
	light_ctrl_instance_t * targetP;
	uint8_t result;
	int i;

	targetP = (light_ctrl_instance_t *) lwm2m_list_find(objectP->instanceList, instanceId);
	if (NULL == targetP) return COAP_404_NOT_FOUND;

	// is the server asking for the full object ?
	if (*numDataP == 0)
	{
		uint16_t resList[] = {
			RES_M_ON_OFF,
			RES_M_DIMMER,
			RES_M_ON_TIME,
			RES_M_COLOUR,
			RES_M_SENSOR_UNITS,
			RES_M_CUM_ACTIVE_POWER,
			RES_M_POWER_FACTOR
	    };
		int nbRes = sizeof(resList)/sizeof(uint16_t);

		*dataArrayP = lwm2m_data_new(nbRes);
		if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
		*numDataP = nbRes;
		for (i = 0 ; i < nbRes ; i++)
		{
			(*dataArrayP)[i].id = resList[i];
		}
	}

	i = 0;
	do
	{
		result = prv_set_value((*dataArrayP) + i, targetP);
		i++;
	} while (i < *numDataP && result == COAP_205_CONTENT);

	return result;
}

static uint8_t prv_lightctrl_discover(uint16_t instanceId,
                            		  int * numDataP,
									  lwm2m_data_t ** dataArrayP,
									  lwm2m_object_t * objectP)
{
	uint8_t result;
	int i;

	result = COAP_205_CONTENT;

	// is the server asking for the full object ?
	if (*numDataP == 0)	{
		uint16_t resList[] = {
			RES_M_ON_OFF,
			RES_M_DIMMER,
			RES_M_ON_TIME,
			RES_M_COLOUR,
			RES_M_SENSOR_UNITS,
			RES_M_CUM_ACTIVE_POWER,
			RES_M_POWER_FACTOR
	    };
	    int nbRes = sizeof(resList) / sizeof(uint16_t);

	    *dataArrayP = lwm2m_data_new(nbRes);
	    if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
	    *numDataP = nbRes;
	    for (i = 0; i < nbRes; i++) {
	    	(*dataArrayP)[i].id = resList[i];
	    }
	}
	else {
		for (i = 0; i < *numDataP && result == COAP_205_CONTENT; i++){
			switch ((*dataArrayP)[i].id) {
				case RES_M_ON_OFF:
				case RES_M_DIMMER:
				case RES_M_ON_TIME:
				case RES_M_COLOUR:
			    case RES_M_SENSOR_UNITS:
			    case RES_M_CUM_ACTIVE_POWER:
			    case RES_M_POWER_FACTOR:
					break;
				default:
					result = COAP_404_NOT_FOUND;
			}
		}
	}
	return result;
}

static uint8_t prv_lightctrl_write(uint16_t instanceId,
                         	 	 	int numData,
									lwm2m_data_t * dataArray,
									lwm2m_object_t * objectP)
{
	light_ctrl_instance_t * targetP;
    int i;
    int64_t tempValue;
    uint8_t result;

	targetP = (light_ctrl_instance_t *) lwm2m_list_find(objectP->instanceList, instanceId);
	if (NULL == targetP) return COAP_404_NOT_FOUND;

    i = 0;
    do
    {
        switch (dataArray[i].id)
        {
			case RES_M_ON_OFF:
				if (1 == lwm2m_data_decode_bool(dataArray + i, &(targetP->is_on)))
				{
					if (targetP->is_on){
						//turn on
						set_red  (targetP->colour_red,targetP->dimmer);
						set_green(targetP->colour_green,targetP->dimmer);
						set_blue (targetP->colour_blue,targetP->dimmer);
						targetP->timeon = time(NULL);
						targetP->currentPowerConsumption =
								(PRV_LIGHT_V * PRV_LIGHT_A) * (double) targetP->colour_red / 255 * (double) targetP->dimmer / 100 +
								(PRV_LIGHT_V * PRV_LIGHT_A) * (double) targetP->colour_green / 255 * (double) targetP->dimmer / 100 +
								(PRV_LIGHT_V * PRV_LIGHT_A) * (double) targetP->colour_blue / 255 * (double) targetP->dimmer / 100 ;
						targetP->lastTimeChanged = targetP->timeon;
					}
					else if (!targetP->is_on){
						//turn off
						targetP->cum_active_power += (double)(time(NULL) - targetP->lastTimeChanged) * targetP->currentPowerConsumption;
						set_red  (0,0);
						set_green(0,0);
						set_blue (0,0);
					}
					result = COAP_204_CHANGED;
				}
				else
				{
					result = COAP_400_BAD_REQUEST;
				}
				break;

			case RES_M_DIMMER:
				if (1 == lwm2m_data_decode_int(dataArray + i, &tempValue))
				{
					if (tempValue > 100){
						tempValue = 100;
					}
					else if (tempValue < 0){
						tempValue = 0;
					}
					targetP->dimmer = (uint8_t) tempValue;
					if (targetP->is_on){
						targetP->cum_active_power += (double)(time(NULL) - targetP->lastTimeChanged) * targetP->currentPowerConsumption;
						targetP->lastTimeChanged = time(NULL);
						targetP->currentPowerConsumption =
														(PRV_LIGHT_V * PRV_LIGHT_A) * (double) targetP->colour_red / 255 * (double) targetP->dimmer / 100 +
														(PRV_LIGHT_V * PRV_LIGHT_A) * (double) targetP->colour_green / 255 * (double) targetP->dimmer / 100 +
														(PRV_LIGHT_V * PRV_LIGHT_A) * (double) targetP->colour_blue / 255 * (double) targetP->dimmer / 100 ;
						set_red  (targetP->colour_red,targetP->dimmer);
						set_green(targetP->colour_green,targetP->dimmer);
						set_blue (targetP->colour_blue,targetP->dimmer);
					}
					result = COAP_204_CHANGED;
				}
				else
				{
					result = COAP_400_BAD_REQUEST;
				}
				break;

			case RES_M_ON_TIME:
				if (1 == lwm2m_data_decode_int(dataArray + i, &tempValue)){
					if (tempValue >= 0){
						targetP->cum_active_power += (double)(time(NULL) - targetP->lastTimeChanged) * targetP->currentPowerConsumption;
						targetP->lastTimeChanged = time(NULL);
						targetP->timeon = time(NULL) - tempValue;
						result = COAP_204_CHANGED;
					}
					else{
						result = COAP_400_BAD_REQUEST;
					}
				}
				else
				{
					result = COAP_400_BAD_REQUEST;
				}
				break;

			case RES_M_COLOUR:
	            if (1 == prv_check_valid_colour((char*)dataArray[i].value.asBuffer.buffer, dataArray[i].value.asBuffer.length))
	            {
	                strncpy(targetP->colourCode, (char*)dataArray[i].value.asBuffer.buffer, dataArray[i].value.asBuffer.length);

	                uint32_t colourVal = prv_store_colour_string(targetP->colourCode, &(targetP->colour_red),&(targetP->colour_green),
																&(targetP->colour_blue));
	                if (targetP->is_on){
						targetP->cum_active_power += (double)(time(NULL) - targetP->lastTimeChanged) * targetP->currentPowerConsumption;
						targetP->lastTimeChanged = time(NULL);
						targetP->currentPowerConsumption =
														(PRV_LIGHT_V * PRV_LIGHT_A) * (double) targetP->colour_red / 255 * (double) targetP->dimmer / 100 +
														(PRV_LIGHT_V * PRV_LIGHT_A) * (double) targetP->colour_green / 255 * (double) targetP->dimmer / 100 +
														(PRV_LIGHT_V * PRV_LIGHT_A) * (double) targetP->colour_blue / 255 * (double) targetP->dimmer / 100 ;
	                	set_red  (targetP->colour_red,targetP->dimmer);
						set_green(targetP->colour_green,targetP->dimmer);
						set_blue (targetP->colour_blue,targetP->dimmer);
					}
	                result = COAP_204_CHANGED;
	            }
	            else
	            {
	                result = COAP_400_BAD_REQUEST;
	            }

	            break;

			default:
				result = COAP_405_METHOD_NOT_ALLOWED;
        }

        i++;
    } while (i < numData && result == COAP_204_CHANGED);

    return result;
}

lwm2m_object_t * get_light_ctrl_object(void)
{
    lwm2m_object_t * lightctrlObj;

    lightctrlObj = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != lightctrlObj)
    {
    	light_ctrl_instance_t * lightctrlInstance;

        memset(lightctrlObj, 0, sizeof(lwm2m_object_t));

        // Assigns the appropriate ID
        lightctrlObj->objID = LIGHT_CONTROL_OBJECT_ID;

        // create hardcoded instance
        lightctrlInstance = (light_ctrl_instance_t *)lwm2m_malloc(sizeof(light_ctrl_instance_t));
        if (NULL == lightctrlInstance)
        {
            lwm2m_free(lightctrlObj);
            return NULL;
        }

        memset(lightctrlInstance, 0, sizeof(light_ctrl_instance_t));

        // assign 0 identifier
        lightctrlInstance->instanceId = 0;

        lightctrlInstance->is_on = false;
        lightctrlInstance->colour_red = 0;
        lightctrlInstance->colour_green = 0;
        lightctrlInstance->colour_blue = 0;
        lightctrlInstance->timeon = time(NULL);
        lightctrlInstance->dimmer = 100; //no dimming
        lightctrlInstance->cum_active_power = 0;
        strcpy(lightctrlInstance->colourCode, "#000000"); //black!

        init_rgb_leds();
        set_red  (0,0);
        set_green(0,0);
        set_blue (0,0);

        lightctrlObj->instanceList = LWM2M_LIST_ADD(lightctrlObj->instanceList, lightctrlInstance);

        // private functions and user data allocation
        lightctrlObj->readFunc     = prv_lightctrl_read;
        lightctrlObj->writeFunc    = prv_lightctrl_write;
        lightctrlObj->discoverFunc = prv_lightctrl_discover;

    }
    return lightctrlObj;
}

void free_light_control_object(lwm2m_object_t * object)
{
    LWM2M_LIST_FREE(object->instanceList);
    if (object->userData != NULL)
    {
        lwm2m_free(object->userData);
        object->userData = NULL;
    }
    lwm2m_free(object);
}

