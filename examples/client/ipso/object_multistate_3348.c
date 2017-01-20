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
 * Implements a multistate object according to IPSO Object 3348
 *
 *                          Multiple
 *     Object      |  ID  | Instances | Mandatory |
 *   Multistate    | 3348 |    Yes    |    No     |
 *
 *  Resources:
 *                             Supported    Multiple
 *      Name         |   ID  | Operations | Instances | Mandatory |  Type   |   Range   | Units | Description |
 *  Multistate Input |  5547 |    R       |    No     |    Yes    | Integer |           |       |             |
 *    Application    |  5750 |    R/W     |    No     |    No     | String  |           |       |             |
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "lwm2mclient.h"
#include "liblwm2m.h"

#define BUFFER_LEN 						20

#define MULTISTATE_OBJECT_ID 			3348

#define RES_M_MS_INPUT                  5547
#define RES_M_APPLICATION				5750

typedef struct _multistate_instance_
{
	struct _multistate_instance_ * next;
	uint16_t instanceId;

    int ms_input;
    char application[BUFFER_LEN];
} multistate_instance_t;

static int prv_check_valid_string(char * buffer,
                                  int length)
{
	if (length < BUFFER_LEN){
		return 1;
	}
	return 0;
}

static uint8_t prv_set_value(lwm2m_data_t * dataP,
                             multistate_instance_t * targetP)
{
    // a simple switch structure is used to respond at the specified resource asked
    switch (dataP->id)
    {

    case RES_M_MS_INPUT:
    	targetP->ms_input = get_last_gesture();
        lwm2m_data_encode_int(targetP->ms_input, dataP);
        return COAP_205_CONTENT;

    case RES_M_APPLICATION:
    	lwm2m_data_encode_string(targetP->application, dataP);
        return COAP_205_CONTENT;

    default:
        return COAP_404_NOT_FOUND;
    }
}

static uint8_t prv_multistate_read(uint16_t instanceId,
                        	   	   int * numDataP,
								   lwm2m_data_t ** dataArrayP,
								   lwm2m_object_t * objectP)
{
	multistate_instance_t * targetP;
	uint8_t result;
	int i;

	targetP = (multistate_instance_t *) lwm2m_list_find(objectP->instanceList, instanceId);
	if (NULL == targetP) return COAP_404_NOT_FOUND;

	// is the server asking for the full object ?
	if (*numDataP == 0)
	{
		uint16_t resList[] = {
			RES_M_MS_INPUT,
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

static uint8_t prv_multistate_discover(uint16_t instanceId,
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
			RES_M_MS_INPUT,
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
				case RES_M_MS_INPUT:
				case RES_M_APPLICATION:
					break;
				default:
					result = COAP_404_NOT_FOUND;
			}
		}
	}
	return result;
}

static uint8_t prv_multistate_write(uint16_t instanceId,
                          	 		int numData,
									lwm2m_data_t * dataArray,
									lwm2m_object_t * objectP)
{
	multistate_instance_t * targetP;
    int i;
    uint8_t result;

	targetP = (multistate_instance_t *) lwm2m_list_find(objectP->instanceList, instanceId);
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

			default:
				result = COAP_405_METHOD_NOT_ALLOWED;
        }

        i++;
    } while (i < numData && result == COAP_204_CHANGED);

    return result;
}

lwm2m_object_t * get_multistate_object(void)
{
    lwm2m_object_t * multistateObj;

    multistateObj = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != multistateObj)
    {
    	multistate_instance_t * multistateInstance;

        memset(multistateObj, 0, sizeof(lwm2m_object_t));

        // Assigns the appropriate ID
        multistateObj->objID = MULTISTATE_OBJECT_ID;

        // create hardcoded instance
        multistateInstance = (multistate_instance_t *)lwm2m_malloc(sizeof(multistate_instance_t));
        if (NULL == multistateInstance)
        {
            lwm2m_free(multistateObj);
            return NULL;
        }

        memset(multistateInstance, 0, sizeof(multistate_instance_t));

        // assign 0 identifier
        multistateInstance->instanceId = 0;

        init_gesture_sensor();
        multistateInstance->ms_input = 0;
        strcpy(multistateInstance->application, "Gesture Sensor"); //application type

        multistateObj->instanceList = LWM2M_LIST_ADD(multistateObj->instanceList, multistateInstance);
        // private functions and user data allocation
        multistateObj->readFunc     = prv_multistate_read;
        multistateObj->writeFunc    = prv_multistate_write;
        multistateObj->discoverFunc = prv_multistate_discover;

    }
    return multistateObj;
}

//void multistate_update_readings(lwm2m_object_t * objectP)
//{
//	multistate_instance_t * targetP;
//
//	if(is_gesture_available())
//	{
//		// just first instance!
//		targetP = (multistate_instance_t *) lwm2m_list_find(objectP->instanceList, 0);
//		if (NULL == targetP) {
//			printf("Cannot find default instance 0\r\n");
//			return;
//		}
//		targetP->ms_input = gesture_read();
//	}
//}


void free_multistate_object(lwm2m_object_t * object)
{
	free_gesture_sensor();
    LWM2M_LIST_FREE(object->instanceList);
    if (object->userData != NULL)
    {
        lwm2m_free(object->userData);
        object->userData = NULL;
    }
    lwm2m_free(object);
}

