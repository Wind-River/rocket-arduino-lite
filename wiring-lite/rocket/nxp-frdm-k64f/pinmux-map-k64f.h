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

#ifndef __PINMUX_MAP_K64F_H__
#define __PINMUX_MAP_K64F_H__

#include <drivers/k20_pcr.h>

#define K64F_PIN_ID(pin_port, pin_index) \
	(((pin_port)<<5)|((pin_index)&0x1f))
	
#define K64F_PIN_PORT(pin_id) \
	((pin_id)>>5)

#define K64F_PIN_INDEX(pin_id) \
	((pin_id)&0x1f)

/*PIN which are disabled or not connected*/
#define K64F_PIN_ID_NA (0xff)


//now the ID and index are the same value, but might be changed later:
#define K64F_PIN_ID_2_INDEX(k64f_pin_id) \
		(k64f_pin_id)


	
	
#define _K64F_MUX_INVALID (uint8_t)(-1)
#define _K64F_MUX_DISABLED (0)
#define _K64F_MUX_GPIO (1)


#define _K64F_PIN_PORT_COUNT (PCR_PORT_E+1)


#define FUNC_BITS_PULL_MASK (0xf0)
#define FUNC_BITS_PULLUP (0x10)
#define FUNC_BITS_PULLDOWN (0x20)


enum _port_function_type_t {
	e_K64F_PORT_FUNCTION_GPIO=0,
	e_K64F_PORT_FUNCTION_PWM,
	e_K64F_PORT_FUNCTION_ADC,
	e_K64F_PORT_FUNCTION_SDA,/*I2C*/
	e_K64F_PORT_FUNCTION_SCL,/*I2C*/
	e_K64F_PORT_FUNCTION_RMII,/*ENET*/
	e_K64F_PORT_FUNCTION_SPI_MOSI,/*SPI*/
	e_K64F_PORT_FUNCTION_SPI_SCLK,/*SPI*/
	e_K64F_PORT_FUNCTION_SPI_MISO,/*SPI*/
	e_K64F_PORT_FUNCTION_SPI_SSEL,/*SPI*/
	e_K64F_PORT_FUNCTION_COUNT
};

uint32_t _k64f_get_mux_config(uint8_t k64f_pin_id, enum _port_function_type_t type);


#endif //__PINMUX_MAP_K64F_H__
