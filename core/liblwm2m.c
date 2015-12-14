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
 *    Fabien Fleutot - Please refer to git log
 *    Simon Bernard - Please refer to git log
 *    Toby Jaffey - Please refer to git log
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

#include "internals.h"

#include <stdlib.h>
#include <string.h>

#include <stdio.h>


lwm2m_context_t * lwm2m_init(lwm2m_connect_server_callback_t connectCallback,
        lwm2m_buffer_send_callback_t bufferSendCallback,
        void * userData)
{
    lwm2m_context_t * contextP;

    if (NULL == bufferSendCallback)
        return NULL;

#ifdef LWM2M_CLIENT_MODE
    if (NULL == connectCallback)
        return NULL;
#endif

    contextP = (lwm2m_context_t *)lwm2m_malloc(sizeof(lwm2m_context_t));
    if (NULL != contextP)
    {
        memset(contextP, 0, sizeof(lwm2m_context_t));
        contextP->connectCallback = connectCallback;
        contextP->bufferSendCallback = bufferSendCallback;
        contextP->userData = userData;
        srand(time(NULL));
        contextP->nextMID = rand();
    }

    return contextP;
}

#ifdef LWM2M_CLIENT_MODE
void lwm2m_deregister(lwm2m_context_t * context)
{
    lwm2m_server_t * server = context->serverList;
    while (NULL != server)
    {
        registration_deregister(context, server);
        server = server->next;
    }
}

void delete_server_list(lwm2m_context_t * context)
{
    while (NULL != context->serverList)
    {
        lwm2m_server_t * server;
        server = context->serverList;
        context->serverList = server->next;
        if (NULL != server->location)
        {
            lwm2m_free(server->location);
        }
        lwm2m_free(server);
    }
}

void delete_bootstrap_server_list(lwm2m_context_t * contextP)
{
    LWM2M_LIST_FREE(contextP->bootstrapServerList);
    contextP->bootstrapServerList = NULL;
}

void delete_observed_list(lwm2m_context_t * contextP)
{
    while (NULL != contextP->observedList)
    {
        lwm2m_observed_t * targetP;

        targetP = contextP->observedList;
        contextP->observedList = contextP->observedList->next;

        LWM2M_LIST_FREE(targetP->watcherList);

        lwm2m_free(targetP);
    }
}
#endif

void delete_transaction_list(lwm2m_context_t * context)
{
    while (NULL != context->transactionList)
    {
        lwm2m_transaction_t * transaction;

        transaction = context->transactionList;
        context->transactionList = context->transactionList->next;
        transaction_free(transaction);
    }
}

void lwm2m_close(lwm2m_context_t * contextP)
{
#ifdef LWM2M_CLIENT_MODE
    int i;

    lwm2m_deregister(contextP);
    delete_server_list(contextP);
    delete_bootstrap_server_list(contextP);
    delete_observed_list(contextP);
    lwm2m_free(contextP->objectList);
    lwm2m_free(contextP->endpointName);
    if (contextP->msisdn != NULL)
    {
        lwm2m_free(contextP->msisdn);
    }
    if (contextP->altPath != NULL)
    {
        lwm2m_free(contextP->altPath);
    }

#endif

#ifdef LWM2M_SERVER_MODE
    while (NULL != contextP->clientList)
    {
        lwm2m_client_t * clientP;

        clientP = contextP->clientList;
        contextP->clientList = contextP->clientList->next;

        prv_freeClient(clientP);
    }
#endif

    delete_transaction_list(contextP);
    lwm2m_free(contextP);
}

#ifdef LWM2M_CLIENT_MODE
static int refresh_server_list(lwm2m_context_t * contextP)
{
    // TODO Improve this to keep on-going operations when bootstrap just adds a new server
    int result;
    bool cleanup = (NULL != contextP->bootstrapServerList) || (NULL != contextP->serverList);
    delete_transaction_list(contextP);
    delete_observed_list(contextP);
    if (cleanup)
    {
        LOG("refresh_server_list: cleanup\n");
        delete_server_list(contextP);
        delete_bootstrap_server_list(contextP);
    }
    result = object_getServers(contextP);
    if (0 != result)
    {
        LOG("refresh_server_list: security- or server-objects configuration error.\n");
        if (cleanup)
        {
            LOG("refresh_server_list: cleanup on error\n");
            delete_server_list(contextP);
            delete_bootstrap_server_list(contextP);
        }
    }
    return result;
}

int lwm2m_configure(lwm2m_context_t * contextP,
                    const char * endpointName,
                    const char * msisdn,
                    const char * altPath,
                    uint16_t numObject,
                    lwm2m_object_t * objectList[])
{
    int i;
    uint8_t found;

    // This API can be called only once for now
    if (contextP->endpointName != NULL) return COAP_400_BAD_REQUEST;

    if (endpointName == NULL) return COAP_400_BAD_REQUEST;
    if (numObject < 3) return COAP_400_BAD_REQUEST;
    // Check that mandatory objects are present
    found = 0;
    for (i = 0 ; i < numObject ; i++)
    {
        if (objectList[i]->objID == LWM2M_SECURITY_OBJECT_ID) found |= 0x01;
        if (objectList[i]->objID == LWM2M_SERVER_OBJECT_ID) found |= 0x02;
        if (objectList[i]->objID == LWM2M_DEVICE_OBJECT_ID) found |= 0x04;
    }
    if (found != 0x07) return COAP_400_BAD_REQUEST;
    if (altPath != NULL)
    {
        if (0 == prv_isAltPathValid(altPath))
        {
            return COAP_400_BAD_REQUEST;
        }
        if (altPath[1] == 0)
        {
            altPath = NULL;
        }
    }
    contextP->endpointName = lwm2m_strdup(endpointName);
    if (contextP->endpointName == NULL)
    {
        return COAP_500_INTERNAL_SERVER_ERROR;
    }

    if (msisdn != NULL)
    {
        contextP->msisdn = lwm2m_strdup(msisdn);
        if (contextP->msisdn == NULL)
        {
            return COAP_500_INTERNAL_SERVER_ERROR;
        }
    }

    if (altPath != NULL)
    {
        contextP->altPath = lwm2m_strdup(altPath);
        if (contextP->altPath == NULL)
        {
            return COAP_500_INTERNAL_SERVER_ERROR;
        }
    }

    contextP->objectList = (lwm2m_object_t **)lwm2m_malloc(numObject * sizeof(lwm2m_object_t *));
    if (NULL != contextP->objectList)
    {
        memcpy(contextP->objectList, objectList, numObject * sizeof(lwm2m_object_t *));
        contextP->numObject = numObject;
    }
    else
    {
        lwm2m_free(contextP->endpointName);
        contextP->endpointName = NULL;
        return COAP_500_INTERNAL_SERVER_ERROR;
    }

    return refresh_server_list(contextP);
}
#endif


int lwm2m_step(lwm2m_context_t * contextP,
               time_t * timeoutP)
{
    time_t tv_sec;
#ifdef LWM2M_SERVER_MODE
    lwm2m_client_t * clientP;
#endif

    tv_sec = lwm2m_gettime();
    if (tv_sec < 0) return COAP_500_INTERNAL_SERVER_ERROR;

#ifdef LWM2M_CLIENT_MODE
    switch (contextP->state)
    {
    case STATE_INITIAL:
        if (contextP->serverList != NULL)
        {
            contextP->state = STATE_REGISTER_REQUIRED;
        }
        else if (contextP->bootstrapServerList != NULL)
        {
            // Bootstrapping
            contextP->state = STATE_BOOTSTRAP_REQUIRED;
        }
        else
        {
            // No server
            return COAP_503_SERVICE_UNAVAILABLE;
        }
        *timeoutP = 0;
        break;

    case STATE_BOOTSTRAP_REQUIRED:
        bootstrap_start(contextP);
        contextP->state = STATE_BOOTSTRAPPING;
        bootstrap_step(contextP, tv_sec, timeoutP);
        break;

    case STATE_BOOTSTRAPPING:
        switch (bootstrap_get_status(contextP))
        {
        case STATE_BS_FINISHED:
            refresh_server_list(contextP);
            contextP->state = STATE_REGISTER_REQUIRED;
            *timeoutP = 0;
            break;

        case STATE_BS_FAILED:
            return COAP_503_SERVICE_UNAVAILABLE;

        default:
            // keep on waiting
            bootstrap_step(contextP, tv_sec, timeoutP);
            break;
        }
        break;

    case STATE_REGISTER_REQUIRED:
        registration_start(contextP);
        contextP->state = STATE_REGISTERING;
        break;

    case STATE_REGISTERING:
    {
        switch (registration_get_status(contextP))
        {
        case STATE_REGISTERED:
            contextP->state = STATE_READY;
            break;

        case STATE_REG_FAILED:
            // TODO avoid infinite loop by checking the bootstrap info is different
            contextP->state = STATE_BOOTSTRAP_REQUIRED;
            *timeoutP = 0;
            break;

        case STATE_REG_PENDING:
        default:
            // keep on waiting
            break;
        }
    }
    break;

    case STATE_READY:
    default:
        // do nothing
        break;
    }

    registration_step(contextP, tv_sec, timeoutP);
#endif

#ifdef LWM2M_SERVER_MODE
    // monitor clients lifetime
    clientP = contextP->clientList;
    while (clientP != NULL)
    {
        lwm2m_client_t * nextP = clientP->next;

        if (clientP->endOfLife <= tv_sec)
        {
            contextP->clientList = (lwm2m_client_t *)LWM2M_LIST_RM(contextP->clientList, clientP->internalID, NULL);
            if (contextP->monitorCallback != NULL)
            {
                contextP->monitorCallback(clientP->internalID, NULL, DELETED_2_02, LWM2M_CONTENT_TEXT, NULL, 0, contextP->monitorUserData);
            }
            prv_freeClient(clientP);
        }
        else
        {
            time_t interval;

            interval = clientP->endOfLife - tv_sec;

            if (*timeoutP > interval)
            {
                *timeoutP = interval;
            }
        }
        clientP = nextP;
    }
#endif

    transaction_step(contextP, tv_sec, timeoutP);

    return 0;
}
