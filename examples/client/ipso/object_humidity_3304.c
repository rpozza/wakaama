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
 * Implements a humidity object according to IPSO Object 3304
 *
 *                          Multiple
 *     Object      |  ID  | Instances | Mandatory |
 *    Humidity     | 3304 |    Yes    |    No     |
 *
 *  Resources:
 *                             Supported    Multiple
 *      Name         |   ID  | Operations | Instances | Mandatory |  Type   |   Range   | Units | Description |
 *  Min. Meas. Value |  5601 |    R       |    No     |    No     |  Float  |           |       |             |
 *  Max. Meas. Value |  5602 |    R       |    No     |    No     |  Float  |           |       |             |
 *  Min. Range Value |  5603 |    R       |    No     |    No     |  Float  |           |       |             |
 *  Max  Range Value |  5604 |    R       |    No     |    No     |  Float  |           |       |             |
 *  Reset Min & Max  |  5605 |    E       |    No     |    No     | Opaque  |           |       |             |
 *    Sensor Value   |  5700 |    R       |    No     |   Yes     |  Float  |           |       |             |
 *    Sensor Units   |  5701 |    R       |    No     |    No     | String  |           |       |             |
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "lwm2mclient.h"
#include "liblwm2m.h"

#define HUMIDITY_OBJECT_ID 				3304

#define PRV_MIN_VALUE		 			0
#define PRV_MAX_VALUE		 			100
#define PRV_SENS_UNIT					"percentage"

#define RES_M_MIN_MEASURED_VALUE        5601
#define RES_M_MAX_MEASURED_VALUE        5602
#define RES_M_MIN_RANGE_VALUE        	5603
#define RES_M_MAX_RANGE_VALUE        	5604
#define RES_M_RESET_VALUES	            5605
#define RES_M_READ_VALUE                5700
#define RES_M_READ_UNIT                	5701

typedef struct _humidity_instance_
{
	struct _humidity_instance_ * next;
	uint16_t instanceId;

    double min_value;
    double max_value;
    double last_value;
} humidity_instance_t;

static double prv_from_int(unsigned int input){
	double retval = 0;
    retval = input / (double)65536;
    retval = -6 + (125 * retval);
    return retval;
}

static uint8_t prv_set_value(lwm2m_data_t * dataP,
                             humidity_instance_t * targetP)
{
	double valueread;
    // a simple switch structure is used to respond at the specified resource asked
    switch (dataP->id)
    {
    case RES_M_MIN_MEASURED_VALUE:
        lwm2m_data_encode_float(targetP->min_value, dataP);
        return COAP_205_CONTENT;

    case RES_M_MAX_MEASURED_VALUE:
        lwm2m_data_encode_float(targetP->max_value, dataP);
        return COAP_205_CONTENT;

    case RES_M_MIN_RANGE_VALUE:
        lwm2m_data_encode_float(PRV_MIN_VALUE, dataP);
        return COAP_205_CONTENT;

    case RES_M_MAX_RANGE_VALUE:
        lwm2m_data_encode_float(PRV_MAX_VALUE, dataP);
        return COAP_205_CONTENT;

    case RES_M_READ_VALUE:
    {
    	valueread = prv_from_int(get_raw_humidity());
    	if (valueread < targetP->min_value){
    		targetP->min_value = valueread;
    	}
    	if (valueread > targetP->max_value){
    		targetP->max_value = valueread;
    	}
    	targetP->last_value = valueread;
    	lwm2m_data_encode_float(targetP->last_value, dataP);
        return COAP_205_CONTENT;
    }

    case RES_M_READ_UNIT:
    	lwm2m_data_encode_string(PRV_SENS_UNIT, dataP);
        return COAP_205_CONTENT;

    default:
        return COAP_404_NOT_FOUND;
    }
}

static uint8_t prv_humidity_read(uint16_t instanceId,
                        	     int * numDataP,
								 lwm2m_data_t ** dataArrayP,
								 lwm2m_object_t * objectP)
{
	humidity_instance_t * targetP;
	uint8_t result;
	int i;

	targetP = (humidity_instance_t *) lwm2m_list_find(objectP->instanceList, instanceId);
	if (NULL == targetP) return COAP_404_NOT_FOUND;

	// is the server asking for the full object ?
	if (*numDataP == 0)
	{
		uint16_t resList[] = {
			RES_M_MIN_MEASURED_VALUE,
			RES_M_MAX_MEASURED_VALUE,
			RES_M_MIN_RANGE_VALUE,
			RES_M_MAX_RANGE_VALUE,
//			E: RES_M_RESET_VALUES,
			RES_M_READ_VALUE,
			RES_M_READ_UNIT
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

static uint8_t prv_humidity_discover(uint16_t instanceId,
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
			RES_M_MIN_MEASURED_VALUE,
			RES_M_MAX_MEASURED_VALUE,
			RES_M_MIN_RANGE_VALUE,
			RES_M_MAX_RANGE_VALUE,
			RES_M_RESET_VALUES,
			RES_M_READ_VALUE,
			RES_M_READ_UNIT
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
				case RES_M_MIN_MEASURED_VALUE:
				case RES_M_MAX_MEASURED_VALUE:
				case RES_M_MIN_RANGE_VALUE:
				case RES_M_MAX_RANGE_VALUE:
				case RES_M_RESET_VALUES:
				case RES_M_READ_VALUE:
				case RES_M_READ_UNIT:
					break;
				default:
					result = COAP_404_NOT_FOUND;
			}
		}
	}
	return result;
}

static uint8_t prv_humidity_execute(uint16_t instanceId,
                                  	uint16_t resourceId,
									uint8_t * buffer,
									int length,
									lwm2m_object_t * objectP)

{
    humidity_instance_t * targetP;
    double valueReset;

    targetP = (humidity_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    switch (resourceId)
    {
    case RES_M_RESET_VALUES:
    {
    	valueReset = prv_from_int(get_raw_humidity());
    	targetP->min_value = valueReset;
		targetP->max_value = valueReset;
		targetP->last_value = valueReset;
        return COAP_204_CHANGED;
    }
    default:
        return COAP_405_METHOD_NOT_ALLOWED;
    }
}


lwm2m_object_t * get_humidity_object(void)
{
    lwm2m_object_t * humidityObj;
    double init_humidity_Value;

    humidityObj = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != humidityObj)
    {
    	humidity_instance_t * humidityInstance;

        memset(humidityObj, 0, sizeof(lwm2m_object_t));

        // Assigns the appropriate ID
        humidityObj->objID = HUMIDITY_OBJECT_ID;

        // create hardcoded instance
        humidityInstance = (humidity_instance_t *)lwm2m_malloc(sizeof(humidity_instance_t));
        if (NULL == humidityInstance)
        {
            lwm2m_free(humidityObj);
            return NULL;
        }

        memset(humidityInstance, 0, sizeof(humidity_instance_t));

        // assign 0 identifier
        humidityInstance->instanceId = 0;

        init_temp_humd();
        init_humidity_Value = prv_from_int(get_raw_humidity());

        humidityInstance->last_value = init_humidity_Value;
        humidityInstance->min_value = init_humidity_Value;
        humidityInstance->max_value = init_humidity_Value;

        humidityObj->instanceList = LWM2M_LIST_ADD(humidityObj->instanceList, humidityInstance);
        // private functions and user data allocation
        humidityObj->readFunc     = prv_humidity_read;
        humidityObj->executeFunc  = prv_humidity_execute;
        humidityObj->discoverFunc = prv_humidity_discover;

    }
    return humidityObj;
}

void free_humidity_object(lwm2m_object_t * object)
{
    LWM2M_LIST_FREE(object->instanceList);
    if (object->userData != NULL)
    {
        lwm2m_free(object->userData);
        object->userData = NULL;
    }
    lwm2m_free(object);
}

