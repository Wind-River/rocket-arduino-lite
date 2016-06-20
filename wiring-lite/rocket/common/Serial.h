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

#ifndef _SERIAL_H_
#define _SERIAL_H_
#include "zephyr.h"

#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#else
#error This module requires CONFIG_STDOUT_CONSOLE
#endif


#define BIN   2
#define OCT   8
#define DEC   10
#define HEX   16

class SerialClass {
    public:
        SerialClass() { }
        int print (int p, int nbase = DEC);
        int print (double p, int places = 2) { return(printf("%.*f", places, p)); }
        int print (char p) { return(printf("%c", p)); }
        int print (const char *p) { return(printf("%s", p)); }
        int println() {	return(printf("\n")); }
        int println (int q) { return(print(q) + println()); }
        int println (int p, int nbase) { return(print(p, nbase) + println()); }
        int println (double p, int places = 2) { return(print(p, places) + println()); }
        int println (char p) { return(print(p) + println()); }
        int println (const char *p) { return(print(p) + println()); }
  
};

extern SerialClass Serial;


#endif /* _SERIAL_H_ */
