/*
 * Copyright (c) 2015 Wind River Systems, Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at 
 * 
 * http://www.apache.org/licenses/LICENSE-2.0 
 * 
 * Unless required by applicable law or agreed to in writing, software distributed 
 * under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES 
 * OR CONDITIONS OF ANY KIND, either express or implied. See the License for
 * the specific language governing permissions and limitations under the License. 
 */
#include "BluemixClient.h"
#include "Serial.h"
#include <string.h>


const char BluemixClientClass::pubTopic[] = "iot-2/evt/status/fmt/json";
const char BluemixClientClass::subTopic[] = "iot-2/cmd/+/fmt/json";

/**
 *
 * @brief   Datapoint constructor for float
 *
 *
 * \NOMANUAL
 */
Datapoint::Datapoint(const char name[], float* data) : t(eFloat), n(name), d((void*)data)  {  
}

/**
 *
 * @brief   Datapoint constructor for int
 *
 *
 * \NOMANUAL
 */
Datapoint::Datapoint(const char name[], int* data) : t(eInt), n(name), d((void*)data) {  
    
}

/**
 *
 * @brief   Datapoint constructor for string
 *
 *
 * \NOMANUAL
 */
Datapoint::Datapoint(const char name[], char* data) : t(eString), n(name), d((void*)data) {  
}

/**
 *
 * @brief   BluemixClientClass constructor for registered mode.
 *
 * @param id        Unique id
 * @param type      Type 
 * @param org       Organization
 * @param token     Security token
 * @callback        subscription callback function
 *
 * \NOMANUAL
 */
BluemixClientClass::BluemixClientClass(char* id, char* type, char* org, char* token, void (*callback)(char*, byte*, unsigned int)) : quickstart(false), isConnected(false), deviceID(id), deviceType(type), deviceOrg(org), deviceToken(token) {
}

/**
 *
 * @brief   BluemixClientClass constructor for quickstart.
 * @param id        Unique id
 * 
 *
 * \NOMANUAL
 */
BluemixClientClass::BluemixClientClass(char* id) : quickstart(true), isConnected(false), deviceID(id), deviceType("quickstart"), deviceOrg("quickstart"), deviceToken(NULL) {
}

/**
 *
 * @brief   User API to connect to Bluemix or Bluemix Quickstart.
 *
 * @return  true if conneccted

 * \NOMANUAL
 */
boolean BluemixClientClass::connect() {
    
    boolean rc = false;
  
    if (!connected()) {
        if (connectToBluemix()) {
            isConnected = true;
            rc = true;
        }
    }
    
   return (rc);
}

/**
 *
 * @brief User API to determine connection state of Bluemix
 *
 *
 * @return true if connected, else false
 *
 * \NOMANUAL
 */
boolean BluemixClientClass::connected() {
    return(isConnected);
}

/**
 *
 * @brief User API to publish datapoints to Bluemix
 * 
 * @param d     arrary of datapoints.
 * @param num   number of datapoints in the array
 *
 * @return  true if datapoints were sent
 *
 * \NOMANUAL
 */
boolean BluemixClientClass::publish(Datapoint* d [], int num)  {
 
   boolean rc = false;
   
    if (connected()) {
        formatJsonPayload(payload, d, num);
    
       if (sendToBluemix(payload)) {
           rc = true;
       }    
    }
	
 return 0;
}

/**
 *
 * @brief   Create the JSON payload of the datapoints to sne
 * 
 * @param payload   payload arrary buffer.
 * @param d         array of datapoints.
 * @param num       number of datapoints.
 *
 * @return  true if datapoints were sent
 *
 * \NOMANUAL
 */
void BluemixClientClass::formatJsonPayload(char payload1[], Datapoint* d [], int num)  {
    int index = 0;
    Datapoint* datapoint;
 
    sprintf(payload1,"{\"d\":{");
    index = strlen(payload1);    
    
    for (int i=0; i<num; i++) { 
        datapoint = d[i];
        switch (datapoint->t) {
        case Datapoint::eInt:
            sprintf(&payload1[index], "\"%s\":%d", datapoint->n, *(int*)datapoint->d);
        break; 
         case Datapoint::eFloat:
            sprintf(&payload1[index], "\"%s\":%0.2f", datapoint->n, *(float*)datapoint->d);
        break;
        case Datapoint::eString:
            sprintf(&payload1[index], "\"%s\":\"%s\"",datapoint->n, (char*)datapoint->d);
        break;        
        }
        
        index = strlen(payload1);    

        if (i < (num-1)) {
            sprintf(&payload1[index], ",");
            index = strlen(payload1);    
        }
    }
    sprintf(&payload1[index],"}}");
} 

/**
 *
 * @brief   Create the JSON connection string.
 *
 * \NOMANUAL
 */
void BluemixClientClass::formatJsonConnectionId(char id[]) {
    sprintf(id,"d:%s:%s:%s", deviceOrg, deviceType, deviceID);      
}

/**
 *
 * @brief Returns number of bytes in connnection Id
 * 
 * @return  Number of bytes
 *
 * \NOMANUAL
 */
int BluemixClientClass::getConnectionIdLength()  {
    return(strlen("d:")+strlen(deviceOrg)+strlen(":")+strlen(deviceType)+strlen(":")+strlen(deviceID)+1);
}
