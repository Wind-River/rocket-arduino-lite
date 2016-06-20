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
 *  - EthernetClass.h
 * Defines abstract class EthernetClass
 */
 
#ifndef _ETHERNET_CLASS_H_
#define _ETHERNET_CLASS_H_


class IPAddress {
    public:
        IPAddress(byte a, byte b, byte c, byte d) { address.octet[0] = a; address.octet[1] = b; address.octet[2] = c; address.octet[3] = d; }
        IPAddress(uint32_t ip) { address.val = ip; }
    private:
        union {
            byte octet[4];
            uint32_t val;
        } address;
        
    friend class EthernetClass;
    friend class EthernetClient;
};

class EthernetClass { 
    public:        
        virtual void begin(byte* mac) = 0;
        virtual void begin(byte* mac, byte* ip) = 0;
        virtual void begin(byte* mac, byte* ip, byte* dns) = 0;
        virtual void begin(byte* mac, byte* ip, byte* dns, byte* gateway) = 0;
        virtual void begin(byte* mac, byte* ip, byte* dns, byte* gateway, byte* subnet) = 0;   
        virtual void service() = 0;
        virtual bool connected() = 0;       
        virtual void getMacAddress(byte address[]) = 0;

        
};

#endif /* _ETHERNET_CLASS_H_ */
