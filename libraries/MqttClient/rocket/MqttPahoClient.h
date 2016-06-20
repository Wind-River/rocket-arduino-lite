/*
 * Copyright (c) 2016 Wind River Systems, Inc.
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
 
/* 
 * DESCRIPTION
 *  - MqttPahoClient.h
 * Defines concrete class MqttPahoClient
 */
 
#ifndef _MQTT_PAHO_CLIENT_H
#define _MQTT_PAHO_CLIENT_H
#include "MqttClientClass.h"

extern "C" {
#include "MQTTClient.h"
}

class MqttPahoClient : public MqttClient {
    public:
        MqttPahoClient();
        virtual boolean connect(char* id);
        virtual boolean connect(char* id, char* user, char* pass);
        virtual boolean connect(char* id, char* willTopic, uint8_t willQos, boolean willRetain, char* willMessage);
        virtual boolean connect(char* id, char* user, char* pass, char* willTopic, uint8_t willQos, boolean willRetain, char* willMessage);
        virtual void setCallback(void (*)(char*, byte*, unsigned int));
        virtual boolean subscribe(const char* topic, uint8_t qos = 0); 
        virtual boolean publish(const char* topic,char* payload, uint8_t qos = 0);
        virtual void setServer(char * ip, uint16_t port);
        virtual void loop();
        virtual boolean connected();
    private:
        static MqttPahoClient *singleton;
        static MqttPahoClient* getInstance();
        unsigned char sendbuf[1000];
        unsigned char readbuf[1000];
        void (*callback)(char* topic, byte* payload, unsigned int length);
        static void handler(struct MessageData* data);	
        boolean connect();
        Client pahoClientData;
        Network network;
        MQTTPacket_connectData connectData;
        boolean isConnected;
};

#endif /* _MQTT_PAHO_CLIENT_H */
