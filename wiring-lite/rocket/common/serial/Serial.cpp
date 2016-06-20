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

#include "Serial.h"

SerialClass Serial;

/**
 *
 * @brief Prints an integer with a specific base
 *
 * 
 * @param p		    The value.
 * @param nbae      The base to print.
 * 
 * @return          Number of characters printed.
 *
 * \NOMANUAL
 */
int SerialClass::print (int p, int nbase) {
	switch (nbase) {
	case HEX:
		return printf("%X", p);
		break;
	case OCT:
		return printf("%o", p);
		break;
	case BIN:
		/* method from http://stackoverflow.com/questions/111928/is-there-a-printf-converter-to-print-in-binary-format */
		static char b[33];
		b[32] = '\0';
		for (int z = 0; z < 32; z++) {
			b[31-z] = ((p>>z) & 0x1) ? '1' : '0';
		}
		return printf("%s", b);
		break;
	default:
		return printf("%d", p);
		break;
	}
}