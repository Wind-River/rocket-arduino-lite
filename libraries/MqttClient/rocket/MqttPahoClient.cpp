

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

#include <stdarg.h>
#include <string.h>
#include <microkernel.h>
#include "zephyr.h"
#include "Ethernet.h"
#include "MqttPahoClient.h"

#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#define PRINT           printf
#else
#include <misc/PRINT.h>
#define PRINT           printk
#endif



static char server[200];;
static int serverPort = 0;

MqttPahoClient* MqttPahoClient::singleton = NULL;

/*******************************************************************************
* @brief    Returns the MqttPahoClient singleton.
*
* @return  The singleton instance.
*
* \NOMANUAL
*/
MqttPahoClient* MqttPahoClient::getInstance() {
    return(singleton);
}

/**
 *
 * @brief   MqttPahoClient constructor
 *
 *
 * \NOMANUAL
 */
MqttPahoClient::MqttPahoClient() : MqttClient(), callback(0), connectData(MQTTPacket_connectData_initializer), isConnected(false) {
    network.client = (void*)&Ethernet_client;
    network.mqttread = &mqtt_read;
    network.mqttwrite = &mqtt_write;
    MQTTClient(&pahoClientData, &network, 5000, sendbuf, 1000, readbuf, 1000);
    singleton = this;
}


/**
 *
 * @brief  subscribe handler passed to Paho Client.  Calls Arduino handler.
 *
 * 
 * @param data      MQTT packet date
 *
 *
 * \NOMANUAL
 */
void MqttPahoClient::handler(struct MessageData* data) {
    int i;
    int topicLen =  MQTTstrlen(*(data->topicName));
    int messageLen = data->message->payloadlen;
    char topic[topicLen+1];
    byte* payload = (byte*)(data->message)->payload;
    MqttPahoClient* instance = getInstance();

    if (data->topicName->cstring) {
        strcpy(topic,data->topicName->cstring);
    } else {
        for (i=0; i < topicLen; i++) {
            topic[i] = data->topicName->lenstring.data[i];
        }
        topic[i] = (char)0;
    }
    if (instance) {
        if (instance->callback) {
            instance->callback(topic, payload, messageLen);
        }
    }
}

/**
 *
 * @brief  Sets the subscribe callback
 *
 * 
 * @param cd        Pointer to callback function
 *
 *
 * \NOMANUAL
 */
void MqttPahoClient::setCallback(void (*cb)(char*, byte*, unsigned int)) {
    callback = cb;
}

/*******************************************************************************
* @brief    Connects to an MQTT broker
*
* @param id             unique id of connection
* @param user           user name
* @param pass           password
* @param willtopic      Will topic for connection
* @param willQos        Will QOS
* @param willRetain     Will Retain flag
* @param willMessage    Will message
*
* @return  true is success, false if failure
*
* \NOMANUAL
*/
 boolean MqttPahoClient::connect(char* id, char* user, char* pass, char* willTopic, uint8_t willQos, boolean willRetain, char* willMessage) {

    connectData.clientID.cstring = id;
    connectData.username.cstring = user;
    connectData.password.cstring = pass;
    connectData.will.topicName.cstring = willTopic;
    connectData.will.qos = willQos;
    connectData.will.retained = willRetain;
    connectData.will.message.cstring = willMessage;
    connectData.cleansession = 1;

    connectData.MQTTVersion = 3;

    printf("connect(%s,%s,%s,%s,%d,%d,%s)\n", id, user, pass, willTopic, willQos, willRetain, willMessage);

   return(connect());
}

/*******************************************************************************
* @brief    Connects to an MQTT broker
*
* @param id             unique id of connection
* @param willtopic      Will topic for connection
* @param willQos        Will QOS
* @param willRetain     Will Retain flag
* @param willMessage    Will message
*
* @return  true is success, false if failure
*
* \NOMANUAL
*/
 boolean MqttPahoClient::connect(char* id, char* willTopic, uint8_t willQos, boolean willRetain, char* willMessage) {

    connectData.clientID.cstring = id;
    connectData.will.topicName.cstring = willTopic;
    connectData.will.qos = willQos;
    connectData.will.retained = willRetain;
    connectData.will.message.cstring = willMessage;
    connectData.cleansession = 1;

    connectData.MQTTVersion = 3;
   
    printf("connect(%s,%s,%d,%d,%s)\n", id, willTopic, willQos, willRetain, willMessage);

    
   return(connect());
}

/*******************************************************************************
* @brief    Connects to an MQTT broker
*
* @param id             unique id of connection
* @param user           user name
* @param pass           password
*
* @return  true is success, false if failure
*
* \NOMANUAL
*/
boolean MqttPahoClient::connect(char* id, char* user, char* pass) {

    connectData.clientID.cstring = id;
    connectData.username.cstring = user;
    connectData.password.cstring = pass;
    connectData.cleansession = 1;

    connectData.MQTTVersion = 3;
   
    printf("connect(%s,%s,%s)\n", id, user, pass);
    
   return(connect());
}

/*******************************************************************************
* @brief    Connects to an MQTT broker
*
* @param id             unique id of connection
*
* @return  true is success, false if failure
*
* \NOMANUAL
*/
boolean MqttPahoClient::connect(char* id) {

    connectData.clientID.cstring = id;
    connectData.cleansession = 1;
   
    connectData.MQTTVersion = 3;
   
    printf("connect(%s)\n", id);
    
   return(connect());
}
/*******************************************************************************
* @brief    Connects to an MQTT broker
*
*
* @return  true is success, false if failure
*
* \NOMANUAL
*/
boolean MqttPahoClient::connect() {
    
    if (serverPort != 0) {
        if (!Ethernet_client.connected()) {
            // connect to the server port if we are not already
            if (Ethernet_client.connect(server, serverPort) == 1) { // SUCCESS
                printf("Ethernet client is connected\n");
                // now connect to MQTT broker
                if (!MQTTConnect (&pahoClientData, &connectData)) {
                    isConnected = true;
                }
            }
        } else {
           // we are connected to server port, try to connect to MQTT broker
            
            if (!MQTTConnect (&pahoClientData, &connectData)) {
                isConnected = true;
            }   
        }
    }
    return(isConnected);
}

/*******************************************************************************
* @brief    Subsribes to an MQTT topic
*
* @param topic          MQTT topic string
* @param qos            QoS value for subscribe
*
* @return  true is success, false if failure
*
* \NOMANUAL
*/
boolean MqttPahoClient::subscribe(const char* topic, uint8_t qos)  {
     
    if (!MQTTSubscribe (&pahoClientData, topic, QoS(qos), handler)) {
        return(true);
    } else {
        return(false);
    }   
 }
 
 /*******************************************************************************
* @brief    Publishes to an MQTT topic
*
* @param topic          MQTT topic string
* @param payload        message
* @param qos            QoS value for publish
*
* @return  true is success, false if failure
*
* \NOMANUAL
*/
boolean MqttPahoClient::publish(const char* topic, char* payload, uint8_t qos) {
    struct MQTTMessage msg;
     
    msg.qos = QoS(qos);
    msg.payload = payload;
    msg.payloadlen = strlen(payload);
    PRINT("publishing topic %s payload %s\n", topic, payload);
    if (!MQTTPublish (&pahoClientData, topic, &msg)) {
        return(true);
    } else {
        return(false);
    }
 }
 
 
 /*******************************************************************************
* @brief        Configures server IP and port number for MQTT connection
*
* @param ip          IP of Broker
* @param port        TCP port number of Broker        
*
*
* \NOMANUAL
*/
void MqttPahoClient::setServer(char* ip, uint16_t port) {
    strcpy(server, ip);
    serverPort = port;
}

/*******************************************************************************
* @brief    Kicks the Ethernet driver and Paho Client
*
*
* \NOMANUAL
*/

void MqttPahoClient::loop() {
    // the following should really be a call to the ethernet client
    Ethernet.service();
    MQTTYield(&pahoClientData, 50);;
}

/*******************************************************************************
* @brief    Checks if we are connected
*
*
* @return  true if connected, false if not
*
* \NOMANUAL
*/
boolean MqttPahoClient::connected() {
    return(isConnected);
}


