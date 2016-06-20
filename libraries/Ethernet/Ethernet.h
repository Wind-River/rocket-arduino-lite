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
 *  - Ethernet.h
 * Top level platform specific file for Arduino-lite Ethernet
 */
 
#ifndef _ETHERNET_H_
#define _ETHERNET_H_

#include "LwipEthernetClient.h"

extern LwipEthernetClient Ethernet_client;


#ifdef CONFIG_BOARD_GALILEO

#include "GmacClass.h"

extern GmacClass Ethernet;
    
#endif /* CONFIG_BOARD_GALILEO */


#ifdef CONFIG_BOARD_FRDM_K64F

#include "FecClass.h"

extern FecClass Ethernet;
    
#endif /* CONFIG_BOARD_FRDM_K64F */


#endif /* _ETHERNET_H */
