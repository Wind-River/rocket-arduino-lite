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
 *  - LwipEthernetClass.h
 * Defines abstract class LwipEthernetClass
 */
 
#ifndef _LWIP_ETHERNET_CLASS_H_
#define _LWIP_ETHERNET_CLASS_H_

#include "EthernetClass.h"
#include <lwip/netif.h>

class LwipEthernetClass : public EthernetClass { 
    public:
        LwipEthernetClass();
        void begin(); 
        virtual void begin(byte* mac);
        virtual void begin(byte* mac, byte* ip);
        virtual void begin(byte* mac, byte* ip, byte* dns);
        virtual void begin(byte* mac, byte* ip, byte* dns, byte* gateway);
        virtual void begin(byte* mac, byte* ip, byte* dns, byte* gateway, byte* subnet);     
        virtual void service();
        virtual bool connected();
     
    protected:
            static struct netif* eth;
    private:
        virtual void kickDriver() = 0;
        virtual struct netif* getEthernetDev() = 0 ;
        virtual err_t (*getInitFunction())(struct netif *netif) = 0;
        virtual void* getConfig() = 0;
        void setCallbacks();
        void waitForLinkUp(); 
        static ip4_addr_t  ipaddr;
        static ip4_addr_t  netmask;
        static ip4_addr_t  gw;
        static boolean isUp;
        static void netif_combined_callback(struct netif *netif);
};

#endif /* _LWIP_ETHERNET_H_ */
