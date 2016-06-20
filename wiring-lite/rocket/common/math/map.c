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

#include "Arduino-lite.h"



#if 0
/*******************************************************************************
* map - maps the input value to another output value range
*
* Arguments: value     - input value
*            fromLow   - low end of source range
*            fromHigh  - high end of source range
*            toLow     - low end of destination range
*            toHigh    - high end of destination range
*
* Returns:  int32_t - mapped value
*/
int32_t map(int32_t value, int32_t fromLow, int32_t fromHigh, \
			int32_t toLow, int32_t toHigh)
{
	int32_t _fromLow = min(fromLow, fromHigh);
	int32_t _fromHigh = max(fromLow, fromHigh);
	int32_t _toLow = min(toLow, toHigh);
	int32_t _toHigh = max(toLow, toHigh);
	
	return (value - _fromLow) * (_toHigh - _toLow) / (_fromHigh - _fromLow) + _toLow;
}
#else
/*******************************************************************************
* map - maps the input value to another output value range
*
* Arguments: value     - input value
*            fromLow   - low end of source range
*            fromHigh  - high end of source range
*            toLow     - low end of destination range
*            toHigh    - high end of destination range
*
* Returns:  uint32_t - mapped value
*/
int32_t map(int32_t value, int32_t fromLow, int32_t fromHigh, \
             int32_t toLow,int32_t toHigh)
{
	return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

#endif
