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


#include <stdio.h>
#include <stdarg.h>
#include "Ethernet.h"
#include <string.h>
#include <microkernel.h>
#include "zephyr.h"

#ifdef CONFIG_BOARD_GALILEO

#include <drivers/pci/pci.h>
#include <drivers/ioapic.h>

#endif /* CONFIG_BOARD_GALILEO */

#include <lwip/init.h>
#include <lwip/timers.h>
#include <lwip/raw.h>
#include <lwip/inet.h>
#include <lwip/netif.h>
#include <lwip/dhcp.h>
#include <netif/etharp.h>

boolean LwipEthernetClass::isUp = false;
struct netif* LwipEthernetClass::eth = NULL;
ip4_addr_t LwipEthernetClass::ipaddr;
ip4_addr_t LwipEthernetClass::netmask;
ip4_addr_t LwipEthernetClass::gw;

/**
 *
 * @brief   lwipEthernetClass constructor
 *
 * \NOMANUAL
 */
LwipEthernetClass::LwipEthernetClass() : EthernetClass() {
    lwip_init();
}
/**
 *
 * @brief    returns the current clock tick for lwip         
 *
 *
 * @return   current tick
 *
 * \NOMANUAL
 */
extern "C" u32_t sys_now() {
    return (sys_tick_get()*1000)/sys_clock_ticks_per_sec;
}

/**
 *
 * @brief  Light Weight IP callout for state of ethernet interface
 *
 * @param netif The inteface in queston 
 * *
 * \NOMANUAL
 */
void LwipEthernetClass::netif_combined_callback(struct netif *netif) {
    if (netif_is_up(netif) && netif_is_link_up(netif)) {
        if (strcmp("0.0.0.0", ipaddr_ntoa(&netif->ip_addr)) != 0) {
            PRINT("Link is up\n");
            PRINT("IP address: %s\n",
            ipaddr_ntoa(&netif->ip_addr));
            isUp = true;
        } /* End of strncmp */
    } else {
        PRINT("Link is down\n");
        isUp = false;
    }
}

/**
 *
 * @brief           prints the hexidecimal value of each nibble in a byte
 *
 * @param val       value of char to print in hex       
 * *
 * \NOMANUAL
 */
static void printhex(byte val) {
    char leftNibble;
    char rightNibble; 

    leftNibble = (val >> 4) & 0xf;
    leftNibble = (leftNibble < 10) ? (leftNibble + '0') : (leftNibble - 10 + 'A');
    rightNibble = val & 0xf;
    rightNibble = (rightNibble < 10) ? (rightNibble + '0') : (rightNibble - 10 + 'A');
    PRINT("%c%c", leftNibble, rightNibble);
}

/**
 *
 * @brief Passes the L1 and L2 callback functions to Light Weight IP
 *
 * \NOMANUAL
 */
void LwipEthernetClass::setCallbacks() {
    eth = netif_add(getEthernetDev(), NULL, NULL, NULL, getConfig(), getInitFunction(), ethernet_input);

    netif_set_link_callback(eth, netif_combined_callback);
    netif_set_status_callback(eth, netif_combined_callback);
}

/**
 *
 * @brief Brings up an ethernetinterface with DHCP
 * 
 * \NOMANUAL
 */
void LwipEthernetClass::begin() {
    setCallbacks();
//    PRINT("mac=");
//    printhex(mac[0]); PRINT("."); printhex(mac[1]); PRINT("."); printhex(mac[2]); PRINT("."); printhex(mac[3]); PRINT("."); printhex(mac[4]); PRINT("."); printhex(mac[5]); PRINT("\n");
    PRINT("dhcping ip\n");     
    netif_set_up(eth);  
    netif_set_default(eth);
    dhcp_start(eth);   
    waitForLinkUp();
}

/**
 *
 * @brief Brings up an ethernetinterface with DHCP
 * 
 * @param netif     MAC address (ignored)
 *
 * \NOMANUAL
 */
void LwipEthernetClass::begin(byte* mac) {
    setCallbacks();
    PRINT("mac=");
    printhex(mac[0]); PRINT("."); printhex(mac[1]); PRINT("."); printhex(mac[2]); PRINT("."); printhex(mac[3]); PRINT("."); printhex(mac[4]); PRINT("."); printhex(mac[5]); PRINT("\n");
    PRINT("dhcping ip\n");     
    netif_set_up(eth);  
    netif_set_default(eth);
    dhcp_start(eth);   
    waitForLinkUp();
}

/**
 *
 * @brief Brings up an ethernetinterface with a static IP address
 * 
 * @param netif     MAC address (ignored)
 * @param ip        A static IP
 *
 * \NOMANUAL
 */
void LwipEthernetClass::begin(byte* mac, byte* ip) {
    setCallbacks();
    PRINT("ip=");
    printhex(ip[0]); PRINT("."); printhex(ip[1]); PRINT("."); printhex(ip[2]); PRINT("."); printhex(ip[3]); PRINT("\n");    
    IP4_ADDR(&ipaddr, ip[0], ip[1], ip[2], ip[3]); 
    netif_set_addr(eth, &ipaddr, NULL, NULL);    
    netif_set_default(eth);
    netif_set_up(eth);      
    waitForLinkUp();
}

/**
 *
 * @brief Brings up an ethernetinterface with a static IP address
 * 
 * @param netif     MAC address (ignored)
 * @param ip        A static IP
 * @param dns       The dns server (ignored)
 *
 * \NOMANUAL
 */
void LwipEthernetClass::begin(byte* mac, byte* ip, byte* dns) { 
    setCallbacks();
    PRINT("ip=");
    printhex(ip[0]); PRINT("."); printhex(ip[1]); PRINT("."); printhex(ip[2]); PRINT("."); printhex(ip[3]); PRINT("\n");    
    IP4_ADDR(&ipaddr, ip[0], ip[1], ip[2], ip[3]); 
    PRINT("dns=");
    printhex(dns[0]); PRINT("."); printhex(dns[1]); PRINT("."); printhex(dns[2]); PRINT("."); printhex(dns[3]); PRINT("\n");
    netif_set_addr(eth, &ipaddr, NULL, NULL);    
    netif_set_up(eth);      
    waitForLinkUp();
}

/**
 *
 * @brief Brings up an ethernetinterface with a static IP address
 * 
 * @param netif     MAC address (ignored)
 * @param ip        A static IP
 * @param dns       The dns server (ignored) 
 * @param gateway   The gateway address (ignored)
 *
 * \NOMANUAL
 */
void LwipEthernetClass::begin(byte* mac, byte* ip, byte* dns, byte* gateway) {
     setCallbacks();
    PRINT("ip=");
     printhex(ip[0]); PRINT("."); printhex(ip[1]); PRINT("."); printhex(ip[2]); PRINT("."); printhex(ip[3]); PRINT("\n");    
     IP4_ADDR(&ipaddr, ip[0], ip[1], ip[2], ip[3]); 
     PRINT("dns=");
     printhex(dns[0]); PRINT("."); printhex(dns[1]); PRINT("."); printhex(dns[2]); PRINT("."); printhex(dns[3]); PRINT("\n");
     PRINT("gateway=");
     printhex(gateway[0]); PRINT("."); printhex(gateway[1]); PRINT("."); printhex(gateway[2]); PRINT("."); printhex(gateway[3]); PRINT("\n");
    IP4_ADDR(&gw, ip[0], ip[1], ip[2], ip[3]); 
     netif_set_addr(eth, &ipaddr, &gw, NULL);    
     netif_set_up(eth);      
     waitForLinkUp();
}

/**
 *
 * @brief Brings up an ethernetinterface with a static IP address
 * 
 * @param netif     MAC address (ignored)
 * @param ip        A static IP
 * @param dns       The dns server (ignored) 
 * @param gateway   The gateway address (ignored)
 * @param subnet    Network subnet mask
 *
 * \NOMANUAL
 */
void LwipEthernetClass::begin(byte* mac, byte* ip, byte* dns, byte* gateway, byte* subnet) {
    setCallbacks();
    PRINT("ip=");
    printhex(ip[0]); PRINT("."); printhex(ip[1]); PRINT("."); printhex(ip[2]); PRINT("."); printhex(ip[3]); PRINT("\n");    
    IP4_ADDR(&ipaddr, ip[0], ip[1], ip[2], ip[3]); 
    PRINT("dns=");
    printhex(dns[0]); PRINT("."); printhex(dns[1]); PRINT("."); printhex(dns[2]); PRINT("."); printhex(dns[3]); PRINT("\n");
    PRINT("gateway=");
    printhex(gateway[0]); PRINT("."); printhex(gateway[1]); PRINT("."); printhex(gateway[2]); PRINT("."); printhex(gateway[3]); PRINT("\n");
    IP4_ADDR(&gw, ip[0], ip[1], ip[2], ip[3]); 
    PRINT("mask=");
    printhex(subnet[0]); PRINT("."); printhex(subnet[1]); PRINT("."); printhex(subnet[2]); PRINT("."); printhex(subnet[3]); ; PRINT("\n");    
    IP4_ADDR(&netmask, subnet[0], subnet[1], subnet[2], subnet[3]);   
    netif_set_addr(eth, &ipaddr, &netmask, &gw); 
    netif_set_up(eth);
    waitForLinkUp();
}

 /**
 *
 * @brief Wait, idefinitely for a NIC to come up
 *
 * \NOMANUAL
 */
void LwipEthernetClass::waitForLinkUp() {
    do {
        /*
         * Use polling to service the ethernet driver
         */

        service();

        task_sleep(1);

    } while (!connected());   
    netif_set_default(eth);     
}

 /**
 *
 * @brief Service the NIC
 *
 * \NOMANUAL
 */
void LwipEthernetClass::service() {
    sys_check_timeouts();
    kickDriver();
}

 /**
 *
 * @brief returns the status of connection
 * 
 * @return          true if connected else false
 *
 * \NOMANUAL
 */
bool LwipEthernetClass::connected() {
    return(isUp);
}
