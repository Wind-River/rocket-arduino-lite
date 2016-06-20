/*
 * Copyright (c) 2016, Wind River Systems, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/* 
 * DESCRIPTION
 *  - BluemixClient.h
 * Defines abstract BluemixClientClass
 */

#ifndef _BLUEMIX_CLIENT_H_
#define _BLUEMIX_CLIENT_H_

class Datapoint {
    public:
        Datapoint(const char name[], float* data);
        Datapoint(const char name[], int* data);   
        Datapoint(const char name[], char* data);
        void show();
        enum {
            eFloat,
            eInt,
            eString
        } t;
        const char* n;
        void*  d;
 };

class BluemixClientClass {

    public:
        BluemixClientClass(char* id);
        BluemixClientClass(char* id, char* type, char* org, char* token, void (*callback)(char*, byte*, unsigned int));
        boolean connected();
        boolean connect();
        boolean publish(Datapoint* d[], int num);

    protected:
        void formatJsonPayload(char payload1[], Datapoint* d [], int num);
        void formatJsonConnectionId(char id[]);
        int getConnectionIdLength();
        virtual boolean connectToBluemix() = 0;
        virtual boolean sendToBluemix(char payload[]) = 0;
        static const char subTopic[];
        static const char pubTopic[];
        bool quickstart;
        bool isConnected;
        char* deviceID;
        char* deviceType;
        char* deviceOrg;
        char* deviceToken;
        
    private: 
        enum {
            eMaxPayloadSize = 255
        };
        char payload[eMaxPayloadSize];    
};


#endif /* _BLUEMIX_CLIENT_H_ */