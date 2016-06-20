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
 
#ifdef CONFIG_BOARD_FRDM_K64F

#include <stdio.h>
#include <stdarg.h>
#include "FecClass.h"

 /**
 *
 * @brief   FecClass constructor
 *
 *
 * \NOMANUAL
 */
 
FecClass::FecClass() : LwipEthernetClass() {
}

/**
 *
 * @brief   Returns the physical driver used by LWIP.
 *
 * @return  Pointer to the fec driver structure.
 *
 * \NOMANUAL
 */
struct netif* FecClass::getEthernetDev() {
    return(&fec);
} 

/**
 *
 * @brief   Returns the driver's intit function used by LWIP.
 *
 * @return  Pointet to fec_init.
 *
 * \NOMANUAL
 */
err_t (*FecClass::getInitFunction())(struct netif *netif) {
    return(fec_init);
}

/**
 *
 * @brief   Returns the config stucture of the driver used by LWIP.
 *
 * @return  The fec_config stucture.
 *
 * \NOMANUAL
 */
void* FecClass::getConfig() {
    return(&fec_conf);
}

/**
 *
 * @brief   Allows physical driver to process.
 *
 * \NOMANUAL
 */
void FecClass::kickDriver() {
    fec_service(&fec);
}

/**
 *
 * @brief   Returns a byte array of the MAC address.
 *
 * \NOMANUAL
 */
void FecClass::getMacAddress(byte address[]) {
    for (int i=0; i <6; i++) {
        address[i] = eth->hwaddr[i];
    }
}

#endif /*  CONFIG_BOARD_FRDM_K64F */
