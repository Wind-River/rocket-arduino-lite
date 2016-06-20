 /*
  * Copyright (c) 2015 Wind River Systems, Inc.
- *
- * Redistribution and use in source and binary forms, with or without
- * modification, are permitted provided that the following conditions are met:
- *
- * 1) Redistributions of source code must retain the above copyright notice,
- * this list of conditions and the following disclaimer.
- *
- * 2) Redistributions in binary form must reproduce the above copyright notice,
- * this list of conditions and the following disclaimer in the documentation
- * and/or other materials provided with the distribution.
- *
- * 3) Neither the name of Wind River Systems nor the names of its contributors
- * may be used to endorse or promote products derived from this software without
- * specific prior written permission.
- *
- * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
- * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
- * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
- * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
- * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
- * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
- * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
- * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
- * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
- * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
- * POSSIBILITY OF SUCH DAMAGE.
+ * 
+ * Licensed under the Apache License, Version 2.0 (the "License"); 
+ * you may not use this file except in compliance with the License. 
+ * You may obtain a copy of the License at 
+ * 
+ * http://www.apache.org/licenses/LICENSE-2.0 
+ * 
+ * Unless required by applicable law or agreed to in writing, software distributed 
+ * under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES 
+ * OR CONDITIONS OF ANY KIND, either express or implied. See the License for
+ * the specific language governing permissions and limitations under the License. 
  */
#ifndef __MQTT_VIPER_
#define __MQTT_VIPER_

#include <microkernel.h>
#include "zephyr.h"



#ifdef  __cplusplus
extern "C" {
#endif

// Timer type and routines required by Paho Embedded C client
typedef struct Timer Timer;
struct Timer {
	uint64_t timeout;
};

char expired(Timer*);
void countdown_ms(Timer*, unsigned int);
void countdown(Timer*, unsigned int);
int left_ms(Timer*);
void InitTimer(Timer*);

// Network type and routines required by Paho Embedded C client
// mqttread and mqttwrite called from structure

typedef struct Network Network;
struct Network
{
   
	int (*mqttread) (Network*, unsigned char*, int, int);
	int (*mqttwrite) (Network*, unsigned char*, int, int);
#if 0
	Ethernet_client_dev* client;
#else
//	Client pahoClientData;
	void* client;
#endif	
};
int mqtt_read(Network*, unsigned char*, int, int);
int mqtt_write(Network*, unsigned char*, int, int);

char expired(Timer*);
void countdown_ms(Timer*, unsigned int);
void countdown(Timer*, unsigned int);
int left_ms(Timer*);
void InitTimer(Timer*);

#ifdef  __cplusplus
}
#endif

#endif
