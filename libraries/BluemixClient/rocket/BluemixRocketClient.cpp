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
#include "BluemixRocketClient.h"
#include "Serial.h"
#include "MqttClient.h"
#include "Ethernet.h" 
#include <string.h>

char BluemixRocketClientClass::quickstartAddress[]  = "quickstart.messaging.internetofthings.ibmcloud.com";
char BluemixRocketClientClass::bluemixAddress[] = "messaging.internetofthings.ibmcloud.com";
char devId[13];

/**
 *
 * @brief   BluemixRocketClientClass constructor.
 * 
 * @param type      Type 
 * @param org       Organization
 * @param token     Security token
 * @callback        subscription callback function
 *
 * \NOMANUAL
 */
BluemixRocketClientClass::BluemixRocketClientClass(char* type, char* org, char* token, void (*callback)(char*, byte*, unsigned int))  : BluemixClientClass(0, type, org, token, callback) {
    
     char address[strlen(org)+strlen(".")+strlen(bluemixAddress)+1];
      // Set the our function to be called when we received messages we subscribed to
    MQTT_Client.setCallback(callback);
   
     // Set the server and port to connect to
    sprintf(address,"%s.%s",org,bluemixAddress);
    MQTT_Client.setServer(address, 1883);
}

/**
 *
 * @brief   BluemixRocketClientClass constructor for quckstart.
 *
 *
 * \NOMANUAL
 */
BluemixRocketClientClass::BluemixRocketClientClass() : BluemixClientClass(0) {
   
     // Set the server and port to connect to
    MQTT_Client.setServer(quickstartAddress, 1883);
}


/**
 *
 * @brief   Connects to the Bluemix MQTT broker
 *  
 * @return  true if successful connection to Bluemix
 *
 * \NOMANUAL
 */
boolean BluemixRocketClientClass::connectToBluemix () {
    bool rc = false;
    
    printf("About to connect to Bluemix\n");
    if (!Ethernet.connected()) {    
      Ethernet.begin();
    }
    if (Ethernet.connected()) {
        if (!deviceID) { 
            byte mac[6];

            Ethernet.getMacAddress(mac);
            Serial.println("Bluemix using MAC address as device ID");
            sprintf(devId,"%X%X%X%X%X%X%X%X%X%X%X%X", (mac[0] >> 4), (mac[0] & 0xf), (mac[1] >> 4), (mac[1] & 0xf), (mac[2] >> 4), (mac[2] & 0xf), (mac[3] >> 4), (mac[3] & 0xf), (mac[4] >> 4), (mac[4] & 0xf), (mac[5] >> 4), (mac[5] & 0xf));
            deviceID = devId;
        }
        if (!MQTT_Client.connected()) {
            char id[getConnectionIdLength()];
            formatJsonConnectionId(id);
            if (quickstart) {
                if (MQTT_Client.connect(id)) {
                    rc = true;
                }
            } else {
                if (MQTT_Client.connect(id, "use-token-auth", deviceToken)) {           
                    printf("Subscribing to %s\n", subTopic);
                    if (MQTT_Client.subscribe(subTopic)) {
                        rc = true;
                    } 
                }
            }
        }
    }
        
    return(rc);
}


/**
 *
 * @brief   Publishs formatted Jason payload to Bluemix MQTT server
 * 
 * @param payload   JSON payload
 *
 * @return  true if successful connection to Bluemix
 *
 * \NOMANUAL
 */
boolean BluemixRocketClientClass::sendToBluemix(char payload[])  {
    
    boolean rc = false;
    
    if (MQTT_Client.publish(pubTopic, payload)) {
         rc = true;
    } else {
        Serial.println("Failed to send to Bluemix");
    }
    
     MQTT_Client.loop();	

    return(rc);
}



