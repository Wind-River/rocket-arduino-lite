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
#ifdef CONFIG_BOARD_GALILEO

#include <stdio.h>
#include <stdarg.h>
#include "GmacClass.h"

 /**
 *
 * @brief   GmacClass constructor
 *
 *
 * \NOMANUAL
 */
 
GmacClass::GmacClass() : LwipEthernetClass() {
}

/**
 *
 * @brief   Returns the physical driver used by LWIP.
 *
 * @return  Pointer to the gmac driver structure.
 *
 * \NOMANUAL
 */
struct netif* GmacClass::getEthernetDev() {
    return(&gmac);
} 

/**
 *
 * @brief   Returns the driver's intit function used by LWIP.
 *
 * @return  Pointet to gmac_init.
 *
 * \NOMANUAL
 */
err_t (*GmacClass::getInitFunction())(struct netif *netif) {
    return(gmac_init);
}

/**
 *
 * @brief   Returns the config stucture of the driver used by LWIP.
 *
 * @return  The gmac_config stucture.
 *
 * \NOMANUAL
 */
void* GmacClass::getConfig() {
    return(&gmac_conf);
}

/**
 *
 * @brief   Allows physical driver to process.
 *
 * \NOMANUAL
 */
void GmacClass::kickDriver() {
    gmac_service(&gmac);
}

/**
 *
 * @brief   Returns a byte array of the MAC address.
 *
 * \NOMANUAL
 */
void GmacClass::getMacAddress(byte address[]) {
    for (int i=0; i <6; i++) {
        address[i] = eth->hwaddr[i];
    }
}

#endif /* CONFIG_BOARD_GALILEO */
