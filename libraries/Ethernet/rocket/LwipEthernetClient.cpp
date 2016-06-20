

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

#include "Arduino-lite.h"
#include <stdio.h>
#include <stdarg.h>
#include "lwip/tcp.h"
#include <lwip/inet.h>
#include <lwip/netif.h>
#include <lwip/dns.h>
#include <string.h>
#include <microkernel.h>
#include "zephyr.h"
#include "LwipEthernetClient.h"
#include "Ethernet.h"

#define PRINT_MQTT_HEADERS
#ifdef PRINT_MQTT_HEADERS
#include "MQTTPacket.h"
#endif


//  Conflicts with MQTTClient (Paho C client)
enum {
    SUCCESS = 1,
    TIMED_OUT = -1,
    INVALID_SERVER = -2,
    TRUNCATED = -3,
    INVALID_RESPONSE = -4
};

#ifdef PRINT_MQTT_HEADERS
/**
 *
 * @brief Display the MQTT header info
 *
 * @param buf     The MQTT header
 *
 * \NOMANUAL
 */
void printMQTTHeader(char* buf) {

static const char* msgStrings[] = {
    "VARIABLE", "CONNECT", "CONNACK", "PUBLISH", 
	"PUBACK", "PUBREC", "PUBREL", "PUBCOMP", 
	"SUBSCRIBE", "SUBACK", "UNSUBSCRIBE", "UNSUBACK",
    "PINGREQ", "PINGRESP", "DISCONNECT"
};

    MQTTHeader header = {0};
    header.byte = buf[0]; 
    int len = (unsigned char)(buf[1]);
    PRINT("%s dup=%d qos=%d retain=%d len=%d\n", msgStrings[header.bits.type], header.bits.dup, header.bits.qos, header.bits.retain, len);
}
#endif


/**
 *
 * @brief LwipEthernetClient constructor.
 * *
 * \NOMANUAL
 */
LwipEthernetClient::LwipEthernetClient() : EthernetClient(), state(net_disconnected), p(NULL), index(0) {
    memset(&host, 0, sizeof(host));
    pcb = tcp_new();

    if (pcb) {
        tcp_arg(pcb, this);
        tcp_recv(pcb, lwip_recv);
        tcp_err(pcb, lwip_err);
        tcp_sent(pcb, lwip_sent);
    } else {
        PRINT("LwipEthernetClient constuctor: Initialize failed to create pcb\n");
    }
}

/*
 *
 * @brief DNS lookup callout
 *
 * @param name   Host name.
 * @param ipaddr Stucture to plunk the address in.
 * @param arg    The LWIP Client instance.
 * 
 * \NOMANUAL
 */

void LwipEthernetClient::serverFound(const char *name, const ip_addr_t* ipaddr, void *arg) {
    LwipEthernetClient* instance = (LwipEthernetClient*)arg;
    
    if (ipaddr) {
        printf("found %s\n", name);
        memcpy(&(instance->host), ipaddr, sizeof(ip_addr_t));
        instance->state = LwipEthernetClient::net_connecting;
    } else {
        printf("cannot resolve %s\n", name);
        instance->state = LwipEthernetClient::net_disconnected;
    }
}

/**
 *
 * @brief Estabishes a TCP connection to a port on a server
 *
 * 
 * @param addr    server to connect to
 * @param port      port to connect to
 * 
 * @return          SUCCESS or TIMED_OUT
 *
 * \NOMANUAL
 */
 
int LwipEthernetClient::connect(char* addr, int port) {
    err_t err = ERR_OK; 
    int rc = INVALID_RESPONSE;
    
    Ethernet.service();

    if (!connected()) {
        switch(dns_gethostbyname(addr, &host, LwipEthernetClient::serverFound, (void*)this)) {
            case ERR_OK:
                printf("Host %s valid\n", addr);
                state = net_connecting;
            break;
            case ERR_INPROGRESS:
                printf("Looking up host: %s...", addr);
                state = net_resolving;
            break;
                state = net_disconnected;
            default:
            break;
        }
     
        while (state == net_resolving)  {  
            Ethernet.service();
            delay(100);
        } 
     
        if (state == net_connecting) {
            printf("Trying to connect to %s on port %d ...", addr, port);
            err = tcp_connect(pcb, &host, port, lwip_connected);
            if (err) {
                PRINT("tcp_connect failed -> %s\n",lwip_strerr(err));
            } else {
                while (state == net_connecting) {  
                    Ethernet.service();
                    delay(100);
                }
                if (state == net_disconnected) {
                    printf("failed to connect\n");
                    rc = TIMED_OUT;
                } else {
                    printf("connected\n");
                    rc = SUCCESS;
                }
            }
        }   
    }

  return rc;
}


/**
 *
 * @brief Display the relivent data from a pbuf stucture
 *
 * @param pbuf     A pointer to a pbuf
 *
 * \NOMANUAL
 */
void printBuf(struct pbuf* p) {
    int len, i;
    char* payload;
    char val;
    if (p) {
        payload = (char*)(p->payload);
        len = p->len;
        PRINT("mbuf(%u) len=%d tot_len=%d next=%u ref=%d\n", p, len, p->tot_len, p->next, p->ref);
        PRINT("[");
        for (i = 0; i < len; i++) {
            PRINT("  %c", (char)payload[i]);
        }
        PRINT("]\n");
        PRINT("[");
        for (i = 0; i < len; i++) {
            val = (payload[i] >> 4) & 0xf;
            val = (val < 10) ? (val + '0') : (val - 10 + 'A');
            PRINT(" %c", val);
            val = payload[i] & 0xf;
            val = (val < 10) ? (val + '0') : (val - 10 + 'A');
            PRINT("%c", val);
        }
        PRINT("]\n");
    } else {
        PRINT("mbuf(NULL)\n"); 
    }
}


/**
 *
 * @brief disconnects from the server
 *
 * \NOMANUAL
 */
void LwipEthernetClient::disconnect() {
    state = net_disconnected;
    tcp_close(pcb);
}


/**
 *
 * @brief Light weight IP TCP error callout.
 *
 * @param arg    pointer to the LwipEthernetClient instance.
 * @param err    The error.
 *
 * \NOMANUAL
 */
void LwipEthernetClient::lwip_err(void *arg, err_t err) {
    LwipEthernetClient* instance = (LwipEthernetClient*)arg;
    
    PRINT("lwip_err(void *arg=%u err_t err=%d %s)\n", arg, err, lwip_strerr(err));
    if (instance->state == net_connecting) {
        instance->state = net_disconnected;
    }
}


/**
 *
 * @brief Light Weight IP TCP sent callout.
 *
 * @param arg    Pointer to EthernetLwipEthernetClient instance.
 * @param pcb    Pointer to the pcb.
 * @param len    Number of sent bytes acknowledged by far end.
 *
 * @return  ERR_OK
 *
 * \NOMANUAL
 */
err_t LwipEthernetClient::lwip_sent(void *arg, struct tcp_pcb *pcb, u16_t len) {
    int length = len;  
    
    // LwipEthernetClient* instance = (LwipEthernetClient*)arg;
    PRINT("sent acknowledge %d bytes\n",length);
    return ERR_OK;
}


/**
 *
 * @brief Lightweight IP TCP recv callout.
 *
 * @param arg    Pointer to the LwipEthernetClient instance.
 * @param pcb    Pointer to the pcb.
 * @param len    Pointer to the pbuf or head of pbuf chain (or NULL) if the far end disconnected.
 * @param err    The error.
 *
 * @return  ERR_OK
 *
 * \NOMANUAL
 */
err_t LwipEthernetClient::lwip_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    LwipEthernetClient* instance = (LwipEthernetClient*)arg;

    if (p) {
#ifdef PRINT_MQTT_HEADERS
        PRINT("RX: ");
        printMQTTHeader((char *)(p->payload));
#endif
        if (instance->p) {
            pbuf_cat(instance->p, p);
            pbuf_ref(p);
        } else {
            instance->p = p;
        }
   //  printBuf(p);
        tcp_recved(pcb, p->tot_len);
    } else {
        PRINT("remote hung up..");
        instance->disconnect();
    }

    return ERR_OK;
}


/**
 *
 * @brief Displays a relivent info for packet processing.
 *
 * \NOMANUAL
 */
void LwipEthernetClient::print() {
    PRINT("LwipEthernetClient(%u)\n", this);
    PRINT("  pcb=%u\n", pcb);
    PRINT("  host=%u\n", host);
    if (p) {
        PRINT("  p=%u\n", p);
        PRINT("  p->len=%u\n", p->len);
        PRINT("  p->tot_len\n", p->tot_len);
    } else {
        PRINT("  p=NULL\n");
    }
    PRINT("  index=%u\n", index);
}
 
/**
 *
 * @brief Returns the connection state
 *
 * @return  true if connected
 *
 * \NOMANUAL
 */
boolean LwipEthernetClient::connected() {

  return(state == net_connected);
}

/**
 *
 * @brief Light Weight IP TCP connected callout.
 *
 * @param arg    Pointer to to the LwipEthernetClient Instance.
 * @param pcb    Pointer to the pcb.
 * @param err    The error.
 *
 * @return  ERR_OK
 *
 * \NOMANUAL
 */
err_t LwipEthernetClient::lwip_connected(void* arg, struct tcp_pcb* pcb, err_t err) {
    LwipEthernetClient* instance = (LwipEthernetClient*)arg; 
 
     PRINT("TCP connection established\n");
    instance->state = net_connected;

  return ERR_OK;
}

/**
 *
 * @brief Sends bytes over TCP/IP
 *
 * @param buffer    Data buffer.
 * @param length    Number of bytes to send.
 *
 * @return  the number of bytes sent.
 *
 * \NOMANUAL
 */
int LwipEthernetClient::send(char* buffer, int length) {

    u16_t len;
    u16_t max_len;
    int sent = 0;
  
    if (connected()) {

#ifdef PRINT_MQTT_HEADERS
        PRINT("TX: ");
        printMQTTHeader(buffer);
#endif

  // We cannot send more data than space available in the send buffer. 
        max_len = tcp_sndbuf(pcb);
        len = (max_len < length) ? max_len : length;
 
        if (tcp_write(pcb, buffer, len, 0) == ERR_OK) {
            if (tcp_output(pcb) == ERR_OK) {  
                sent = len;
            }
        }

    }
    return(sent);
}
  
 /**
 *
 * @brief User APi to write a number of bytes to the Ethernet Client to be sent over TCP/IP.
 *
 * @param buffer    Data buffer.
 * @param length    Number of bytes to send.
 *
 * @return  the number of bytes sent.
 *
 * \NOMANUAL
 */
int LwipEthernetClient::write(char* buffer, int length) {
   
    return(send(buffer, length));
}
  
/**
 *
 * @brief User APi to write a single character to the Ehternet client to be sent over TCP/IP.
 *
 * @param ch        Character to write..
 *
 * @return  the number of bytes sent.
 *
 * \NOMANUAL
 */
int LwipEthernetClient::write(char ch) {
    char buffer[1] = { ch  };
   
    return(send(buffer, 1));
}  

/**
 *
 * @brief User APi to read the net character available from the Ethernet Client..
 *
 *
 * @return  the character or -1.
 *
 * \NOMANUAL
 */
char LwipEthernetClient::read() {
    char out = -1;
    struct pbuf* pBuf = p;

    Ethernet.service();

    if (pBuf) {
        out = ((char*)(pBuf->payload))[index];
        index++;
        if (index == pBuf->len) {
            index=0;
            p = p->next;
            if (p) {
                pbuf_ref(p);
            }
            pbuf_free(pBuf);
        }
    }
    return out;
}

/**
 *
 * @brief User APi to determine the number of bytes availalbe to read from the Ethernet Client..
 *
 *
 * @return  then number of availale bytes
 *
 * \NOMANUAL
 */
int LwipEthernetClient::available() {

    int availableBytes = 0;
 
    Ethernet.service();
    if (connected()) {
        if (p) {
            availableBytes = p->tot_len - index;
        }
    }
    
    return (availableBytes);
}

