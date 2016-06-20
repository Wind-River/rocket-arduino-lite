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

#ifndef _WIRING_MATH_
#define _WIRING_MATH_
#include <math.h>

#undef min
#define min(x, y) ({ \
    __typeof__ (x) _x = (x); \
    __typeof__ (y) _y = (y); \
    _x < _y ? _x : _y; \
})

#undef max
#define max(x, y) ({ \
    __typeof__ (x) _x = (x); \
    __typeof__ (y) _y = (y); \
    _x > _y ? _x : _y; \
})

#define constrain(x, a, b)	({ \
    __typeof__ (x) _x = (x); \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    ( (_x) < (_a) ? (_a) : ((_x)>(_b)?(_b):(_x)) ); \
})
int32_t map(int32_t value, int32_t fromLow, int32_t fromHigh, \
             int32_t toLow, int32_t toHigh);
#define	sq(x)	((x)*(x))

#if 0
double	copysign(double x, double y);
double	scalbn(double x, int n);
double 	log (double x);
double  fabs(double x);
double 	sqrt(double x);
double  pow(double x, double y);

double	floor(double x);
double  sin(double x);
double  cos(double x);
double  tan(double x);
#endif
#endif
