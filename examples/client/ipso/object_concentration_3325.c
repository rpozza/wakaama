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
 * Implements a concentration object according to IPSO Object 3325
 *
 *                          Multiple
 *     Object      |  ID  | Instances | Mandatory |
 *  Concentration  | 3325 |    Yes    |    No     |
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
 *    Application    |  5750 |    R/W     |    No     |    No     | String  |           |       |             |
 *  Current Calibr.  |  5821 |    R/W     |    No     |    No     |  Float  |           |       |             |
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "lwm2mclient.h"
#include "liblwm2m.h"

#define CONCENTRATION_OBJECT_ID 		3325

#define PRV_SENS_UNIT					"mg/m3"
#define PRV_MIN_VOLTAGE					0.7
#define PRV_MAX_VOLTAGE					3.3
#define BUFFER_LEN 						20

#define RES_M_MIN_MEASURED_VALUE        5601
#define RES_M_MAX_MEASURED_VALUE        5602
#define RES_M_MIN_RANGE_VALUE        	5603
#define RES_M_MAX_RANGE_VALUE        	5604
#define RES_M_RESET_VALUES	            5605
#define RES_M_READ_VALUE                5700
#define RES_M_READ_UNIT                	5701
#define RES_M_APPLICATION               5750
#define RES_M_CALIBRATION               5821

typedef struct _concentration_instance_
{
	struct _concentration_instance_ * next;
	uint16_t instanceId;

    double min_value;
    double max_value;
    double min_range;
    double max_range;
    double calibration;
    double last_value;
    char application[BUFFER_LEN];
} concentration_instance_t;

static int prv_check_valid_string(char * buffer,
                                  int length)
{
	if (length < BUFFER_LEN){
		return 1;
	}
	return 0;
}

static double prv_compute_concentration(unsigned int adc_sample, double calibration){
	double retval = 0;
	double adc_value = 0;

	adc_value = adc_sample / (double) 65536;
	adc_value *= 3.3;
	adc_value -= 0.7;

    retval = calibration * adc_value;
    if (retval < 0){ // needed to avoid negative values
    	retval = 0;
    }
    return retval;
}

static uint8_t prv_set_value(lwm2m_data_t * dataP,
                             concentration_instance_t * targetP)
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
    	lwm2m_data_encode_float(targetP->min_range, dataP);
        return COAP_205_CONTENT;

    case RES_M_MAX_RANGE_VALUE:
    	lwm2m_data_encode_float(targetP->max_range, dataP);
        return COAP_205_CONTENT;

    case RES_M_READ_VALUE:
    {
    	valueread = prv_compute_concentration(sample_dust_adc(),targetP->calibration);
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

    case RES_M_APPLICATION:
    	lwm2m_data_encode_string(targetP->application, dataP);
        return COAP_205_CONTENT;

    case RES_M_CALIBRATION:
    	lwm2m_data_encode_float(targetP->calibration, dataP);
    	return COAP_205_CONTENT;

    default:
        return COAP_404_NOT_FOUND;
    }
}

static uint8_t prv_concentration_read(uint16_t instanceId,
                        	          int * numDataP,
									  lwm2m_data_t ** dataArrayP,
									  lwm2m_object_t * objectP)
{
	concentration_instance_t * targetP;
	uint8_t result;
	int i;

	targetP = (concentration_instance_t *) lwm2m_list_find(objectP->instanceList, instanceId);
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
			RES_M_READ_UNIT,
			RES_M_APPLICATION,
			RES_M_CALIBRATION
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

static uint8_t prv_concentration_discover(uint16_t instanceId,
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
			RES_M_READ_UNIT,
			RES_M_APPLICATION,
			RES_M_CALIBRATION
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
				case RES_M_APPLICATION:
				case RES_M_CALIBRATION:
					break;
				default:
					result = COAP_404_NOT_FOUND;
			}
		}
	}
	return result;
}

static uint8_t prv_concentration_execute(uint16_t instanceId,
                                  	     uint16_t resourceId,
									     uint8_t * buffer,
									     int length,
									     lwm2m_object_t * objectP)

{
    concentration_instance_t * targetP;
    double valueReset;

    targetP = (concentration_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    switch (resourceId)
    {
    case RES_M_RESET_VALUES:
    {
    	valueReset = prv_compute_concentration(sample_dust_adc(),targetP->calibration);
    	targetP->min_value = valueReset;
		targetP->max_value = valueReset;
		targetP->last_value = valueReset;
        return COAP_204_CHANGED;
    }
    default:
        return COAP_405_METHOD_NOT_ALLOWED;
    }
}

static uint8_t prv_concentration_write(uint16_t instanceId,
                          	 		   int numData,
									   lwm2m_data_t * dataArray,
									   lwm2m_object_t * objectP)
{
	concentration_instance_t * targetP;
    int i;
    double calibration_value, valueReset;
    uint8_t result;

	targetP = (concentration_instance_t *) lwm2m_list_find(objectP->instanceList, instanceId);
	if (NULL == targetP) return COAP_404_NOT_FOUND;

    i = 0;
    do
    {
        switch (dataArray[i].id)
        {
			case RES_M_APPLICATION:
	            if (1 == prv_check_valid_string((char*)dataArray[i].value.asBuffer.buffer, dataArray[i].value.asBuffer.length))
	            {
	            	memset(targetP->application, 0, BUFFER_LEN);
	                strncpy(targetP->application,(char*)dataArray[i].value.asBuffer.buffer,dataArray[i].value.asBuffer.length);
	                result = COAP_204_CHANGED;
	            }
	            else
	            {
	                result = COAP_400_BAD_REQUEST;
	            }
				break;

			case RES_M_CALIBRATION:
				if (1 == lwm2m_data_decode_float(dataArray + i, &calibration_value))
					// NB: sometimes result=8 from leshan, with default single value "TLV"
					// To fix select single value "Text"
				{
					targetP->calibration = calibration_value;
					targetP->max_range = prv_compute_concentration(65536,targetP->calibration);
					valueReset = prv_compute_concentration(sample_dust_adc(),targetP->calibration);
					targetP->min_value = valueReset;
					targetP->max_value = valueReset;
					targetP->last_value = valueReset;
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

lwm2m_object_t * get_concentration_object(void)
{
    lwm2m_object_t * concentrationObj;
    double init_concentration_Value;

    concentrationObj = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != concentrationObj)
    {
    	concentration_instance_t * concentrationInstance;

        memset(concentrationObj, 0, sizeof(lwm2m_object_t));

        // Assigns the appropriate ID
        concentrationObj->objID = CONCENTRATION_OBJECT_ID;

        // create hardcoded instance
        concentrationInstance = (concentration_instance_t *)lwm2m_malloc(sizeof(concentration_instance_t));
        if (NULL == concentrationInstance)
        {
            lwm2m_free(concentrationObj);
            return NULL;
        }

        memset(concentrationInstance, 0, sizeof(concentration_instance_t));

        // assign 0 identifier
        concentrationInstance->instanceId = 0;

        init_dust_sensor();
        concentrationInstance->calibration = (0.5 / 2.8);

        init_concentration_Value = prv_compute_concentration(sample_dust_adc(),concentrationInstance->calibration);

        concentrationInstance->min_value = init_concentration_Value;
        concentrationInstance->max_value = init_concentration_Value;
        concentrationInstance->last_value = init_concentration_Value;
        concentrationInstance->min_range = 0;
        concentrationInstance->max_range = prv_compute_concentration(65536,concentrationInstance->calibration);

        strcpy(concentrationInstance->application, "Dust Sensor"); //application type

        concentrationObj->instanceList = LWM2M_LIST_ADD(concentrationObj->instanceList, concentrationInstance);
        // private functions and user data allocation
        concentrationObj->readFunc     = prv_concentration_read;
        concentrationObj->executeFunc  = prv_concentration_execute;
        concentrationObj->writeFunc    = prv_concentration_write;
        concentrationObj->discoverFunc = prv_concentration_discover;

    }
    return concentrationObj;
}

void free_concentration_object(lwm2m_object_t * object)
{
    LWM2M_LIST_FREE(object->instanceList);
    if (object->userData != NULL)
    {
        lwm2m_free(object->userData);
        object->userData = NULL;
    }
    lwm2m_free(object);
}

