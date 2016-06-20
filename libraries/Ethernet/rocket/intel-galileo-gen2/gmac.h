/* gmac.h - public API for GMAC lwIP driver */

/*
 * Copyright (c) 2015, Wind River Systems, Inc. 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at 
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0 
 * 
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the License is distributed on an "AS IS" BASIS, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and 
 * limitations under the License.
 */


#ifndef __INCgmach
#define __INCgmach

#include <autoconf.h>
#include <lwip/err.h>
#include <lwip/netif.h>
#include <drivers/pci/pci.h>

#ifndef GMAC_USE_IRQ
#define GMAC_USE_IRQ 1
#endif
#undef GMAC_USE_IRQ

#ifdef __cplusplus
extern "C"
{
#endif

struct gmac_config {
	struct pci_dev_info	pci;
#if defined(CONFIG_MICROKERNEL)
	kevent_t		event;
#endif	

};

extern err_t gmac_init(struct netif *netif);
extern void gmac_service(struct netif *netif);
extern struct gmac_config gmac_conf;

#ifdef __cplusplus
}
#endif

#endif  /* __INCgmach */
