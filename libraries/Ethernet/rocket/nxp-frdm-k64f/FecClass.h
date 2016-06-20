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
 *  - FecClass.h
 * Defines concrete class FecClass
 */
 

#ifndef _FEC_CLASS_H_
#define _FEC_CLASS_H_

#include "LwipEthernetClass.h"
#include "fec.h"

class FecClass : public LwipEthernetClass { 
    public:
        FecClass();
        virtual void getMacAddress(byte address[]);

    private:  
        void kickDriver();
        struct netif* getEthernetDev();
        err_t (*getInitFunction())(struct netif *netif);
        void* getConfig();
        struct netif fec;
};


#endif /* _FEC_CLASS_H_ */


