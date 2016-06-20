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

/* 
 * DESCRIPTION
 *  - MqttClientClass.h
 * Defines abstract class MqttClient
 */
 
#ifndef _MQTT_CLIENT_CLASS_
#define _MQTT_CLIENT_CLASS_

class MqttClient {
    public:
        virtual boolean connect(char* id) = 0;
        virtual boolean connect(char* id, char* user, char* pass) = 0;
        virtual boolean connect(char* id, char* willTopic, uint8_t willQos, boolean willRetain, char* willMessage) = 0;
        virtual boolean connect(char* id, char* user, char* pass, char* willTopic, uint8_t willQos, boolean willRetain, char* willMessage) = 0;
        virtual void setCallback(void (*)(char*, byte*, unsigned int)) = 0;
        virtual boolean subscribe(const char* topic, uint8_t qos) = 0;
        virtual boolean publish(const char* topic, char* payload, uint8_t qos) = 0;
        virtual void setServer(char * ip, uint16_t port) = 0;
        virtual void loop() = 0;
        virtual boolean connected() = 0;
};

#endif /* _MQTT_CLIENT_CLASS_ */
