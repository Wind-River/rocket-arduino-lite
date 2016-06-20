
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
 *  - LwipEthernetClient.h
 * Defines concrete class LwipEthernetClient
 */

#ifndef _LWIP_ETHERNET_CLIENT_H_
#define _LWIP_ETHERNET_CLIENT_H_

#include "EthernetClient.h"
#include "lwip/tcp.h"

class LwipEthernetClient : public EthernetClient {  
    public:
        LwipEthernetClient();
        virtual int connect(char* ip, int port);
        virtual boolean connected();
        virtual void disconnect();
        virtual int write(char ch);
        virtual int write(char* buffer, int length);
        virtual char read();
        virtual int available();
  
    private:
        enum net_state {
            net_disconnected,
            net_resolving,
            net_connecting,
            net_connected,
            net_disconnecting
        } state;
        int send(char* buffer, int length);
        void print();   
        static void serverFound(const char* name, const ip_addr_t* ipaddr, void* arg);
        static err_t lwip_connected(void* arg, struct tcp_pcb* pcb, err_t err);
        static void  lwip_err(void *arg, err_t err);
        static err_t lwip_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
        static err_t lwip_sent(void *arg, struct tcp_pcb *pcb, u16_t len);
        struct tcp_pcb* pcb;
        struct pbuf* p;
        int index;
        ip_addr_t host;
        int serverPort;
};

#endif /* _LWIP_ETHERNET_CLIENT_H_ */
