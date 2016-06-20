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

#include "paho-rocket.h"
#include "Ethernet.h"

/**
 *
 * @brief Return whether the timer has expired or not; used by MQTT client and this glue logic
 *
 * @param timer     Timer
 *
 * @return 0 on not expired, 1 expired
 *
 * \NOMANUAL
 */
char expired(Timer* timer) {
   char done = (left_ms(timer) == 0);

    if (done) {
 
      }

    return(done);
}


/**
 *
 * @brief Starts a timer countdown by a timeeout in milliseconds; used by MQTT Client and this glue logic
 *
 * NOTE:  The timers are implemented using the tick which typically runs at 10ms which cannot give a 
 *        millisecond resolution but is good enough for this MQTT implementation.  To prevent a timer of
 *        10 ms or less from being expired from the getgo, a tick is added to ensure that 
 *        the routine timer runs for a least a tick meaning that the minimum timeout is greater than 0;
 *        Greater timeouts will have a 10ms resolution. 
 *
 * @param timer    pointer to the timer to start.
 * @paraa timeout  the timeout value in milliseconds.
 *
 * \NOMANUAL
 */
void countdown_ms(Timer* timer, unsigned int timeout) {

    uint64_t timeoutTick;
    uint64_t now = sys_tick_get();

    timeoutTick = now + timeout*sys_clock_ticks_per_sec/1000;
    if (timeoutTick <= now) {
        timeoutTick += 1 ;
    }
    timer->timeout = timeoutTick;
}

/**
 *
 * @brief Starts a timer countdown by a timeeout in seconds; used by MQTT Client and this glue logic
 *
 * @param timer    pointer to the timer to start.
 * @param timeout  the timeout value in seconds.
 *
 * \NOMANUAL
 */
void countdown(Timer* timer, unsigned int timeout) {
    countdown_ms(timer, timeout*1000);
}

/**
 *
 * @brief Determine how many milliseconds to go in the timer ; used by MQTT Client and this glue logic
 *
 * @param timer    pointer to the timer to start.
 *
 * @return Number of milliseconds left. 
 *
 * \NOMANUAL
 */
int left_ms(Timer* timer) {
    uint64_t delta = (int)(timer->timeout - sys_tick_get())*1000/sys_clock_ticks_per_sec;

    return ((delta > 0) ? delta : 0);
}

/**
 *
 * @brief Initializes a timer.
 *
 * @param timer    pointer to the timer to initialize.
 *
 * \NOMANUAL
 */
void InitTimer(Timer* timer) {
    timer->timeout = 0;
}

/**
 *
 * @brief MQTT Paho client write interface
 *
 * @param n             pointer to the Network stucture
 * @param buffer        buffer to write out 
 * @param length        number of bytes to write out
 * @timeout timeout_ms  timeout
 *
 * @return Number of by bytes written out 
 *
 * \NOMANUAL
 */
int mqtt_write(Network* n, unsigned char* buffer, int length, int timeout_ms) {
    EthernetClient* client = (EthernetClient*)n->client;
    
    return(client->write((char*)buffer, length));
}

/**
 *
 * @brief MQTT Paho client read interface
 *
 * @param n             pointer to the Network stucture
 * @param buffer        buffer to read into
 * @param length        number of bytes to into
 * @timeout timeout_ms  timeout
 *
 * @return Number of by bytes read
 *
 * \NOMANUAL
 */
int mqtt_read(Network* n, unsigned char* buffer, int length, int timeout_ms) {
    
    int availableBytes;
    int count;
    Timer t;
    EthernetClient* client = (EthernetClient*)n->client;
  
    InitTimer(&t);
    countdown_ms(&t, timeout_ms);
    
// try to do stuff until timer expires

   while (!(availableBytes = client->available()) && (!expired(&t)));
    
   count = (length < availableBytes) ? length : availableBytes;

    for (int i = 0; i < count; i++) {
        buffer[i] = (unsigned char)client->read();
    }
 
    return count;
}

