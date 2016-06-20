/* fec.h - public API for FEC lwIP driver */

/*
 * Copyright (c) 2016, Wind River Systems, Inc.
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

#ifndef __INCfech
#define __INCfech

#include <microkernel.h>
#include <autoconf.h>
#include <lwip/err.h>
#include <lwip/netif.h>
//#include <task_irq.h>

#ifdef __cplusplus
extern "C"
    {
#endif
#define FEC_USE_IRQ 0
#ifndef FEC_USE_IRQ
#define FEC_USE_IRQ	1
#endif

struct fec_config {
#if defined(CONFIG_MICROKERNEL)
	kevent_t		event;
#endif
};
extern struct fec_config fec_conf;

extern err_t fec_init(struct netif *netif);
#if NO_SYS || !FEC_USE_IRQ
extern void fec_service(struct netif *netif);
#endif

#ifdef __cplusplus
    }
#endif

#endif	/* __INCioapich */
