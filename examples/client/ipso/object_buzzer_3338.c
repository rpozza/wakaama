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
 * Implements a buzzer object according to IPSO Object 3338
 *
 *                          Multiple
 *     Object      |  ID  | Instances | Mandatory |
 *     Buzzer      | 3338 |    Yes    |    No     |
 *
 *  Resources:
 *                          Supported    Multiple
 *      Name      |   ID  | Operations | Instances | Mandatory |  Type   |   Range   | Units | Description |
 *      On/Off    |  5850 |    R/W     |    No     |    Yes    | Boolean |   (0;1)   |       |             |
 *      Dimmer    |  5851 |    R/W     |    No     |    No     | Integer | (0<->100) |   %   |             |
 *     Duration   |  5521 |    R/W     |    No     |    No     |  Float  |           |  sec  |             |
 *  Min. Off Time |  5525 |    R/W     |    No     |    Yes    |  Float  |           |  sec  |             |
 *  Application   |  5750 |    R/W     |    No     |    No     | String  |           |       |             |
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "lwm2mclient.h"
#include "liblwm2m.h"

#define BUFFER_LEN 						20

#define BUZZER_OBJECT_ID 				3338

#define RES_M_ON_OFF                    5850
#define RES_M_DIMMER	                5851
#define RES_M_DURATION                  5521
#define RES_M_MIN_OFF_TIME				5525
#define RES_M_APPLICATION				5750

typedef struct _buzzer_instance_
{
	struct _buzzer_instance_ * next;
	uint16_t instanceId;

    bool is_on;
    uint8_t dimmer;
    double on_time;
    double off_time;
    char application[BUFFER_LEN];
} buzzer_instance_t;

static int prv_check_valid_string(char * buffer,
                                  int length)
{
	if (length < BUFFER_LEN){
		return 1;
	}
	return 0;
}

static unsigned int prv_int_converter(double input, uint8_t dim){
	unsigned int retval = 0;
	if (input > 0){
		retval = (input * 10 * dim); // * 1000 / 100 => * 10
	}
	return retval;
}

static uint8_t prv_set_value(lwm2m_data_t * dataP,
                             buzzer_instance_t * targetP)
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

    case RES_M_DURATION:
        lwm2m_data_encode_float(targetP->on_time, dataP);
        return COAP_205_CONTENT;

    case RES_M_MIN_OFF_TIME:
        lwm2m_data_encode_float(targetP->off_time, dataP);
        return COAP_205_CONTENT;

    case RES_M_APPLICATION:
    	lwm2m_data_encode_string(targetP->application, dataP);
        return COAP_205_CONTENT;

    default:
        return COAP_404_NOT_FOUND;
    }
}

static uint8_t prv_buzzer_read(uint16_t instanceId,
                        	   int * numDataP,
							   lwm2m_data_t ** dataArrayP,
							   lwm2m_object_t * objectP)
{
	buzzer_instance_t * targetP;
	uint8_t result;
	int i;

	targetP = (buzzer_instance_t *) lwm2m_list_find(objectP->instanceList, instanceId);
	if (NULL == targetP) return COAP_404_NOT_FOUND;

	// is the server asking for the full object ?
	if (*numDataP == 0)
	{
		uint16_t resList[] = {
			RES_M_ON_OFF,
			RES_M_DIMMER,
			RES_M_DURATION,
			RES_M_MIN_OFF_TIME,
			RES_M_APPLICATION
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

static uint8_t prv_buzzer_discover(uint16_t instanceId,
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
			RES_M_DURATION,
			RES_M_MIN_OFF_TIME,
			RES_M_APPLICATION
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
				case RES_M_DURATION:
				case RES_M_MIN_OFF_TIME:
				case RES_M_APPLICATION:
					break;
				default:
					result = COAP_404_NOT_FOUND;
			}
		}
	}
	return result;
}

static uint8_t prv_buzzer_write(uint16_t instanceId,
                          	 	int numData,
								lwm2m_data_t * dataArray,
								lwm2m_object_t * objectP)
{
	buzzer_instance_t * targetP;
    int i;
    int64_t tempValue;
    double tempValuedouble;
    uint8_t result;
    char doublebuf[sizeof(double)];

	targetP = (buzzer_instance_t *) lwm2m_list_find(objectP->instanceList, instanceId);
	if (NULL == targetP) return COAP_404_NOT_FOUND;

    i = 0;
    do
    {
        switch (dataArray[i].id)
        {
			case RES_M_ON_OFF:
				if (1 == lwm2m_data_decode_bool(dataArray + i, &(targetP->is_on)))
				{
					serialize_bool_t(targetP->is_on, OBJ_3338_RES_5850_ADDR);
					if (targetP->is_on){
						//turn on
						detach_buzzer();
						attach_buzzer_on(prv_int_converter(targetP->on_time, targetP->dimmer),
										 prv_int_converter(targetP->off_time, targetP->dimmer));
					}
					else if (!targetP->is_on){
						//turn off
						detach_buzzer();
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
					serialize_uint8_t (targetP->dimmer,OBJ_3338_RES_5851_ADDR);
					if (targetP->is_on){
						detach_buzzer();
						attach_buzzer_on(prv_int_converter(targetP->on_time, targetP->dimmer),
										 prv_int_converter(targetP->off_time, targetP->dimmer));
					}
					result = COAP_204_CHANGED;
				}
				else
				{
					result = COAP_400_BAD_REQUEST;
				}
				break;

			case RES_M_DURATION:
				if (1 == lwm2m_data_decode_float(dataArray + i, &tempValuedouble))
					// NB: sometimes result=8 from leshan, with default single value "TLV"
					// To fix select single value "Text"
				{
					if (tempValuedouble < 0){
						tempValuedouble = 0;
					}
					targetP->on_time = tempValuedouble;
					memcpy(doublebuf,&targetP->on_time,sizeof(double));
					serialize_char_t(doublebuf,sizeof(double), OBJ_3338_RES_5521_ADDR);
					if (targetP->is_on){
						detach_buzzer();
						attach_buzzer_on(prv_int_converter(targetP->on_time, targetP->dimmer),
										 prv_int_converter(targetP->off_time, targetP->dimmer));
					}
					result = COAP_204_CHANGED;
				}
				else
				{
					result = COAP_400_BAD_REQUEST;
				}
				break;

			case RES_M_MIN_OFF_TIME:
				if (1 == lwm2m_data_decode_float(dataArray + i, &tempValuedouble))
					// NB: sometimes result=8 from leshan, with default single value "TLV"
					// To fix select single value "Text"
				{
					if (tempValuedouble < 0){
						tempValuedouble = 0;
					}
					targetP->off_time = tempValuedouble;
					memcpy(doublebuf,&targetP->off_time,sizeof(double));
					serialize_char_t(doublebuf,sizeof(double), OBJ_3338_RES_5525_ADDR);
					if (targetP->is_on){
						detach_buzzer();
						attach_buzzer_on(prv_int_converter(targetP->on_time, targetP->dimmer),
										 prv_int_converter(targetP->off_time, targetP->dimmer));
					}
					result = COAP_204_CHANGED;
				}
				else
				{
					result = COAP_400_BAD_REQUEST;
				}
				break;
			case RES_M_APPLICATION:
	            if (1 == prv_check_valid_string((char*)dataArray[i].value.asBuffer.buffer, dataArray[i].value.asBuffer.length))
	            {
	            	memset(targetP->application, 0, BUFFER_LEN);
	                strncpy(targetP->application,(char*)dataArray[i].value.asBuffer.buffer,dataArray[i].value.asBuffer.length);
	                serialize_char_t(targetP->application,APPLICATION_BUFFER_LEN,OBJ_3338_RES_5750_ADDR);
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

lwm2m_object_t * get_buzzer_object(void)
{
    lwm2m_object_t * buzzerObj;
    char doublebuf[sizeof(double)];

    buzzerObj = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != buzzerObj)
    {
    	buzzer_instance_t * buzzerInstance;

        memset(buzzerObj, 0, sizeof(lwm2m_object_t));

        // Assigns the appropriate ID
        buzzerObj->objID = BUZZER_OBJECT_ID;

        // create hardcoded instance
        buzzerInstance = (buzzer_instance_t *)lwm2m_malloc(sizeof(buzzer_instance_t));
        if (NULL == buzzerInstance)
        {
            lwm2m_free(buzzerObj);
            return NULL;
        }

        memset(buzzerInstance, 0, sizeof(buzzer_instance_t));

        // assign 0 identifier
        buzzerInstance->instanceId = 0;

        init_buzzer();

        buzzerInstance->is_on = deserialize_bool_t (OBJ_3338_RES_5850_ADDR);
        buzzerInstance->dimmer = deserialize_uint8_t (OBJ_3338_RES_5851_ADDR);
        deserialize_char_t(doublebuf, OBJ_3338_RES_5521_ADDR, sizeof(double));
   		memcpy(&buzzerInstance->on_time,doublebuf,sizeof(double));
        deserialize_char_t(doublebuf, OBJ_3338_RES_5525_ADDR, sizeof(double));
   		memcpy(&buzzerInstance->off_time,doublebuf,sizeof(double));
        deserialize_char_t(buzzerInstance->application,OBJ_3338_RES_5750_ADDR,APPLICATION_BUFFER_LEN);

        if (buzzerInstance->is_on){
			detach_buzzer();
			attach_buzzer_on(prv_int_converter(buzzerInstance->on_time, buzzerInstance->dimmer),
							 prv_int_converter(buzzerInstance->off_time, buzzerInstance->dimmer));
		}
		else if (!buzzerInstance->is_on){
			//turn off
			detach_buzzer();
		}

        buzzerObj->instanceList = LWM2M_LIST_ADD(buzzerObj->instanceList, buzzerInstance);
        // private functions and user data allocation
        buzzerObj->readFunc     = prv_buzzer_read;
        buzzerObj->writeFunc    = prv_buzzer_write;
        buzzerObj->discoverFunc = prv_buzzer_discover;

    }
    return buzzerObj;
}

void free_buzzer_object(lwm2m_object_t * object)
{
    LWM2M_LIST_FREE(object->instanceList);
    if (object->userData != NULL)
    {
        lwm2m_free(object->userData);
        object->userData = NULL;
    }
    lwm2m_free(object);
}

