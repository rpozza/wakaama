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
 *    Julien Vermillard - initial implementation
 *    Fabien Fleutot - Please refer to git log
 *    David Navarro, Intel Corporation - Please refer to git log
 *    Bosch Software Innovations GmbH - Please refer to git log
 *    Pascal Rieux - Please refer to git log
 *    
 *******************************************************************************/

/*
 * This object is single instance only, and provide firmware upgrade functionality.
 * Object ID is 5.
 */

/*
 * resources:
 * 0 package                   write
 * 1 package url               write
 * 2 update                    exec
 * 3 state                     read
 * 4 update supported objects  read/write
 * 5 update result             read
 */

#include "liblwm2m.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "storage.h"

// ---- private object "Firmware" specific defines ----
// Resource Id's:
#define RES_M_PACKAGE                   0
#define RES_M_PACKAGE_URI               1
#define RES_M_UPDATE                    2
#define RES_M_STATE                     3
//#define RES_O_UPDATE_SUPPORTED_OBJECTS  4
#define RES_M_UPDATE_RESULT             5
#define RES_O_PKG_NAME                  6
#define RES_O_PKG_VERSION               7

#define URI_BUFFER_LEN 					50
#define SHORT_BUFFER_LEN 				7

#define STATE_IDLE						0

#define RESULT_UPDATED					1

typedef struct
{
    char package_uri[URI_BUFFER_LEN];
    char fw_version[SHORT_BUFFER_LEN];
    char fw_package[SHORT_BUFFER_LEN];

	uint8_t state;
    bool supported;
    uint8_t result;
} firmware_data_t;

static int prv_check_valid_string(char * buffer,
                                  int length)
{
	if (length < URI_BUFFER_LEN){
		return 1;
	}
	return 0;
}

static uint8_t prv_firmware_read(uint16_t instanceId,
                                 int * numDataP,
                                 lwm2m_data_t ** dataArrayP,
                                 lwm2m_object_t * objectP)
{
    int i;
    uint8_t result;
    firmware_data_t * data = (firmware_data_t*)(objectP->userData);

    // this is a single instance object
    if (instanceId != 0)
    {
        return COAP_404_NOT_FOUND;
    }

    // is the server asking for the full object ?
    if (*numDataP == 0)
    {
//        *dataArrayP = lwm2m_data_new(5);
    	*dataArrayP = lwm2m_data_new(4);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
//        *numDataP = 5;
        *numDataP = 4;
        (*dataArrayP)[0].id = RES_M_STATE;
//        (*dataArrayP)[1].id = RES_O_UPDATE_SUPPORTED_OBJECTS;
        (*dataArrayP)[1].id = RES_M_UPDATE_RESULT;
        (*dataArrayP)[2].id = RES_O_PKG_NAME;
        (*dataArrayP)[3].id = RES_O_PKG_VERSION;
    }

    i = 0;
    do
    {
        switch ((*dataArrayP)[i].id)
        {
        case RES_M_PACKAGE:
			result = COAP_405_METHOD_NOT_ALLOWED;
        	break;
        case RES_M_PACKAGE_URI:
        	lwm2m_data_encode_string(data->package_uri, *dataArrayP + i);
            result = COAP_205_CONTENT;
            break;
        case RES_M_UPDATE:
            result = COAP_405_METHOD_NOT_ALLOWED;
            break;

        case RES_M_STATE:
            lwm2m_data_encode_int(data->state, *dataArrayP + i);
            result = COAP_205_CONTENT;
            break;

//        case RES_O_UPDATE_SUPPORTED_OBJECTS:
//            lwm2m_data_encode_bool(data->supported, *dataArrayP + i);
//            result = COAP_205_CONTENT;
//            break;

        case RES_M_UPDATE_RESULT:
            lwm2m_data_encode_int(data->result, *dataArrayP + i);
            result = COAP_205_CONTENT;
            break;

        case RES_O_PKG_NAME:
        	lwm2m_data_encode_string(data->fw_package, *dataArrayP + i);
        	result = COAP_205_CONTENT;
        	break;

        case RES_O_PKG_VERSION:
            lwm2m_data_encode_string(data->fw_version, *dataArrayP + i);
            result = COAP_205_CONTENT;
            break;

        default:
            result = COAP_404_NOT_FOUND;
        }

        i++;
    } while (i < *numDataP && result == COAP_205_CONTENT);

    return result;
}

static uint8_t prv_firmware_write(uint16_t instanceId,
                                  int numData,
                                  lwm2m_data_t * dataArray,
                                  lwm2m_object_t * objectP)
{
    int i;
    uint8_t result;
    firmware_data_t * data = (firmware_data_t*)(objectP->userData);

    // this is a single instance object
    if (instanceId != 0)
    {
        return COAP_404_NOT_FOUND;
    }

    i = 0;

    do
    {
        switch (dataArray[i].id)
        {
        case RES_M_PACKAGE:
            // inline firmware magic code
//        	if (1 == prv_check_full_page((uint8_t*)dataArray[i].value.asBuffer.buffer, dataArray[i].value.asBuffer.length))
//			{
//        		if (data->result == RESULT_BUSY){
//        			//busy? then do nothing
//					if (!flash_is_busy()){
//						data->result = RESULT_UPDATED;
//					}
//				}
//        		if ((data->state == STATE_DOWNLOADING) && (data->result == RESULT_UPDATED)){
//        			// download started
//					memset(data->page_data, 0, PAGE_SIZE);
//					memcpy(data->page_data,(uint8_t*)dataArray[i].value.asBuffer.buffer + ADDRESS_SIZE, dataArray[i].value.asBuffer.length - ADDRESS_SIZE);
//					memcpy(tempaddress,(uint8_t*)dataArray[i].value.asBuffer.buffer, ADDRESS_SIZE);
//					data->page_address = (((uint32_t)tempaddress[0]) << 24) + (((uint32_t)tempaddress[1]) << 16) + (((uint32_t)tempaddress[2]) << 8) + ((uint32_t)tempaddress[3]);
////					fprintf(stderr,"\r\n ADDRESS: %X", data->page_address);
////					fprintf(stderr,"\r\n DATA: ");
////					for (it=0; it<PAGE_SIZE;it++){
////						fprintf(stderr,"%X", data->page_data[it]);
////					}
////					fprintf(stderr,"\r\n");
//					flash_program_page(data->page_data,data->page_address,PAGE_SIZE);
//					data->result = RESULT_BUSY;
//					result = COAP_204_CHANGED;
//        		}
//        		else
//				{
//					result = COAP_400_BAD_REQUEST;
//				}
//        	}
//			else
//			{
//				result = COAP_400_BAD_REQUEST;
//			}
//			break;
//            result = COAP_204_CHANGED;
//            break;

        case RES_M_PACKAGE_URI:
        	if (1 == prv_check_valid_string((char*)dataArray[i].value.asBuffer.buffer, dataArray[i].value.asBuffer.length))
			{
        		memset(data->package_uri, 0, URI_BUFFER_LEN);
        		strncpy(data->package_uri,(char*)dataArray[i].value.asBuffer.buffer,dataArray[i].value.asBuffer.length);
        		lwm2m_store_new_value(package_uri,(void*) package_uri, sizeof(package_uri));
   				result = COAP_204_CHANGED;
			}
			else
			{
				result = COAP_400_BAD_REQUEST;
			}
            break;

//        case RES_O_UPDATE_SUPPORTED_OBJECTS:
//            if (lwm2m_data_decode_bool(&dataArray[i], &data->supported) == 1)
//            {
//                result = COAP_204_CHANGED;
//            }
//            else
//            {
//                result = COAP_400_BAD_REQUEST;
//            }
//            break;

        default:
            result = COAP_405_METHOD_NOT_ALLOWED;
        }

        i++;
    } while (i < numData && result == COAP_204_CHANGED);

    return result;
}

static uint8_t prv_firmware_execute(uint16_t instanceId,
                                    uint16_t resourceId,
                                    uint8_t * buffer,
                                    int length,
                                    lwm2m_object_t * objectP)
{
    firmware_data_t * data = (firmware_data_t*)(objectP->userData);

    // this is a single instance object
    if (instanceId != 0)
    {
        return COAP_404_NOT_FOUND;
    }

    if (length != 0) return COAP_400_BAD_REQUEST;

    // for execute callback, resId is always set.
    switch (resourceId)
    {
    case RES_M_UPDATE:
    	fprintf(stdout, "\n\t FIRMWARE UPDATE!\r\n\n");
        //trigger your firmware download and update logic
    	//TODO::check the magic code is OK!
        system_reboot();
        return COAP_204_CHANGED;
    default:
        return COAP_405_METHOD_NOT_ALLOWED;
    }
}

void display_firmware_object(lwm2m_object_t * object)
{
#ifdef WITH_LOGS
    firmware_data_t * data = (firmware_data_t *)object->userData;
    fprintf(stdout, "  /%u: Firmware object:\r\n", object->objID);
    if (NULL != data)
    {
        fprintf(stdout, "    state: %u, supported: %s, result: %u\r\n",
                data->state, data->supported?"true":"false", data->result);
    }
#endif
}

lwm2m_object_t * get_object_firmware(void)
{
    /*
     * The get_object_firmware function create the object itself and return a pointer to the structure that represent it.
     */
    lwm2m_object_t * firmwareObj;

    firmwareObj = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != firmwareObj)
    {
        memset(firmwareObj, 0, sizeof(lwm2m_object_t));

        /*
         * It assigns its unique ID
         * The 5 is the standard ID for the optional object "Object firmware".
         */
        firmwareObj->objID = LWM2M_FIRMWARE_UPDATE_OBJECT_ID;

        /*
         * and its unique instance
         *
         */
        firmwareObj->instanceList = (lwm2m_list_t *)lwm2m_malloc(sizeof(lwm2m_list_t));
        if (NULL != firmwareObj->instanceList)
        {
            memset(firmwareObj->instanceList, 0, sizeof(lwm2m_list_t));
        }
        else
        {
            lwm2m_free(firmwareObj);
            return NULL;
        }

        /*
         * And the private function that will access the object.
         * Those function will be called when a read/write/execute query is made by the server. In fact the library don't need to
         * know the resources of the object, only the server does.
         */
        firmwareObj->readFunc    = prv_firmware_read;
        firmwareObj->writeFunc   = prv_firmware_write;
        firmwareObj->executeFunc = prv_firmware_execute;
        firmwareObj->userData    = lwm2m_malloc(sizeof(firmware_data_t));

        /*
         * Also some user data can be stored in the object with a private structure containing the needed variables
         */
        if (NULL != firmwareObj->userData)
        {
            ((firmware_data_t*)firmwareObj->userData)->state = STATE_IDLE;
            ((firmware_data_t*)firmwareObj->userData)->supported = false;
            ((firmware_data_t*)firmwareObj->userData)->result = RESULT_UPDATED;
            lwm2m_get_value(package_uri,(void*) ((firmware_data_t*)firmwareObj->userData)->package_uri,
            		sizeof(((firmware_data_t*)firmwareObj->userData)->package_uri));
            lwm2m_get_value(fw_version,(void*) ((firmware_data_t*)firmwareObj->userData)->fw_version,
            		sizeof(((firmware_data_t*)firmwareObj->userData)->fw_version));
            lwm2m_get_value(fw_package,(void*) ((firmware_data_t*)firmwareObj->userData)->fw_package,
            		sizeof(((firmware_data_t*)firmwareObj->userData)->fw_package));
        }
        else
        {
            lwm2m_free(firmwareObj);
            firmwareObj = NULL;
        }
    }

    return firmwareObj;
}

void free_object_firmware(lwm2m_object_t * objectP)
{
    if (NULL != objectP->userData)
    {
        lwm2m_free(objectP->userData);
        objectP->userData = NULL;
    }
    if (NULL != objectP->instanceList)
    {
        lwm2m_free(objectP->instanceList);
        objectP->instanceList = NULL;
    }
    lwm2m_free(objectP);
}

