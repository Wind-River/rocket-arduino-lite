/* k64f_pinmux.c - pin out mapping for the k64f board */

/*
 * Copyright (c) 2015 Intel Corporation
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
#ifdef CONFIG_BOARD_FRDM_K64F

#include <nanokernel.h>
#include <device.h>
#include <init.h>

#include <pinmux.h>

#include <pinmux-map-k64f.h>

struct _mux_config_t
{
	uint8_t pin_id;
	uint8_t mux_val;
};


/*GPIO does not require config table due to fixed mux value: 1*/



/*PWM*/
static const struct _mux_config_t _mux_config_k64_pwm[] = 
{
    {K64F_PIN_ID(PCR_PORT_A,0 ), 3},
    {K64F_PIN_ID(PCR_PORT_A,1 ), 3},
    {K64F_PIN_ID(PCR_PORT_A,2 ), 3},
    {K64F_PIN_ID(PCR_PORT_A,3 ), 3},
    {K64F_PIN_ID(PCR_PORT_A,4 ), 3},
    {K64F_PIN_ID(PCR_PORT_A,5 ), 3},
    {K64F_PIN_ID(PCR_PORT_A,6 ), 3},
    {K64F_PIN_ID(PCR_PORT_A,7 ), 3},
    {K64F_PIN_ID(PCR_PORT_A,8 ), 3},
    {K64F_PIN_ID(PCR_PORT_A,9 ), 3},
    {K64F_PIN_ID(PCR_PORT_A,10), 3},
    {K64F_PIN_ID(PCR_PORT_A,11), 3},
    {K64F_PIN_ID(PCR_PORT_A,12), 3},
    {K64F_PIN_ID(PCR_PORT_A,13), 3},
    
    {K64F_PIN_ID(PCR_PORT_B,0 ), 3},
    {K64F_PIN_ID(PCR_PORT_B,1 ), 3},
    {K64F_PIN_ID(PCR_PORT_B,18), 3},
    {K64F_PIN_ID(PCR_PORT_B,19), 3},
    
    {K64F_PIN_ID(PCR_PORT_C,1 ), 4},
    {K64F_PIN_ID(PCR_PORT_C,2 ), 4},
    {K64F_PIN_ID(PCR_PORT_C,3 ), 4},
    {K64F_PIN_ID(PCR_PORT_C,4 ), 4},
    {K64F_PIN_ID(PCR_PORT_C,5 ), 7},
    {K64F_PIN_ID(PCR_PORT_C,8 ), 3},
    {K64F_PIN_ID(PCR_PORT_C,9 ), 3},
    {K64F_PIN_ID(PCR_PORT_C,10), 3},
    {K64F_PIN_ID(PCR_PORT_C,11), 3},
    
    {K64F_PIN_ID(PCR_PORT_D,0 ), 4},
    {K64F_PIN_ID(PCR_PORT_D,1 ), 4},
    {K64F_PIN_ID(PCR_PORT_D,2 ), 4},
    {K64F_PIN_ID(PCR_PORT_D,3 ), 4},
    {K64F_PIN_ID(PCR_PORT_D,4 ), 4},
    {K64F_PIN_ID(PCR_PORT_D,5 ), 4},
    {K64F_PIN_ID(PCR_PORT_D,6 ), 4},
    {K64F_PIN_ID(PCR_PORT_D,4 ), 4},
    {K64F_PIN_ID(PCR_PORT_D,7 ), 4},
    
    {K64F_PIN_ID(PCR_PORT_E,5 ), 6},
    {K64F_PIN_ID(PCR_PORT_E,6 ), 6},
};
#define _mux_config_k64_pwm_count (sizeof(_mux_config_k64_pwm)/sizeof(struct _mux_config_t))


/*ADC*/
static const struct _mux_config_t _mux_config_k64_adc[] = 
{
    {K64F_PIN_ID(PCR_PORT_A,17), 	0},
		
    {K64F_PIN_ID(PCR_PORT_B,0 ), 	0},
    {K64F_PIN_ID(PCR_PORT_B,1 ), 	0},
    {K64F_PIN_ID(PCR_PORT_B,2 ), 	0},
    {K64F_PIN_ID(PCR_PORT_B,3 ), 	0},
    {K64F_PIN_ID(PCR_PORT_B,10), 	0},
    {K64F_PIN_ID(PCR_PORT_B,11), 	0},
    
    {K64F_PIN_ID(PCR_PORT_C,0 ), 	0},
    {K64F_PIN_ID(PCR_PORT_C,1 ), 	0},
    {K64F_PIN_ID(PCR_PORT_C,2 ), 	0},
    {K64F_PIN_ID(PCR_PORT_C,8 ), 	0},
    {K64F_PIN_ID(PCR_PORT_C,9 ), 	0},
    {K64F_PIN_ID(PCR_PORT_C,10),	0},
    {K64F_PIN_ID(PCR_PORT_C,11), 	0},
    
    {K64F_PIN_ID(PCR_PORT_D,5 ), 	0},
    {K64F_PIN_ID(PCR_PORT_D,6 ), 	0},
    {K64F_PIN_ID(PCR_PORT_D,1 ), 	0},
    
};
#define _mux_config_k64_adc_count (sizeof(_mux_config_k64_adc)/sizeof(struct _mux_config_t))

static const struct _mux_config_t _mux_config_k64_sda[] = {
    {K64F_PIN_ID(PCR_PORT_A,13), 5},
    {K64F_PIN_ID(PCR_PORT_B,1 ), 2},
    {K64F_PIN_ID(PCR_PORT_B,3 ), 2},
    {K64F_PIN_ID(PCR_PORT_C,11), 2},
    {K64F_PIN_ID(PCR_PORT_D,3 ), 7},
    {K64F_PIN_ID(PCR_PORT_E,0 ), 6},
    {K64F_PIN_ID(PCR_PORT_E,25), 5},
};
#define _mux_config_k64_sda_count (sizeof(_mux_config_k64_sda)/sizeof(struct _mux_config_t))

static const struct _mux_config_t _mux_config_k64_scl[] = {
    {K64F_PIN_ID(PCR_PORT_A,12), 5},
    {K64F_PIN_ID(PCR_PORT_A,14), 5},
    {K64F_PIN_ID(PCR_PORT_B,0 ), 2},
    {K64F_PIN_ID(PCR_PORT_B,2 ), 2},
    {K64F_PIN_ID(PCR_PORT_C,10), 2},
    {K64F_PIN_ID(PCR_PORT_D,2 ), 7},
    {K64F_PIN_ID(PCR_PORT_E,1 ), 6},
    {K64F_PIN_ID(PCR_PORT_E,24), 5},
};
#define _mux_config_k64_scl_count (sizeof(_mux_config_k64_scl)/sizeof(struct _mux_config_t))

static const struct _mux_config_t _mux_config_k64_rmii[] = {
    {K64F_PIN_ID(PCR_PORT_A,5 ), 4},/*PTA5 = RMII0_RXER*/
    {K64F_PIN_ID(PCR_PORT_A,12), 4},/*PTA12 = RMII0_RXD1*/
    {K64F_PIN_ID(PCR_PORT_A,13), 4},/*PTA13 = RMII0_RXD0*/
    {K64F_PIN_ID(PCR_PORT_A,14), 4},/*PTA14 = RMII0_CRS_DV*/
    {K64F_PIN_ID(PCR_PORT_A,15), 4},/*PTA15 = RMII0_TXEN*/
    {K64F_PIN_ID(PCR_PORT_A,16), 4},/*PTA16 = RMII0_TXD0*/
    {K64F_PIN_ID(PCR_PORT_A,17), 4},/*PTA17 = RMII0_TXD1*/
    {K64F_PIN_ID(PCR_PORT_A,18), 1},/*PTA18 = EXTAL0 ? */
    {K64F_PIN_ID(PCR_PORT_B,0 ), 4},/*PTB0 = RMII0_MDIO*/
    {K64F_PIN_ID(PCR_PORT_B,1 ), 4},/*PTB1 = RMII0_MDC*/
};
#define _mux_config_k64_rmii_count (sizeof(_mux_config_k64_rmii)/sizeof(struct _mux_config_t))

/* SPI */
static const struct _mux_config_t _mux_config_k64_spi_mosi[] = {
    {K64F_PIN_ID(PCR_PORT_D,2 ), 2},
};
#define _mux_config_k64_spi_mosi_count (sizeof(_mux_config_k64_spi_mosi)/sizeof(struct _mux_config_t))


static const struct _mux_config_t _mux_config_k64_spi_sclk[] = {
    {K64F_PIN_ID(PCR_PORT_D,1 ), 2},
};
#define _mux_config_k64_spi_sclk_count (sizeof(_mux_config_k64_spi_sclk)/sizeof(struct _mux_config_t))


static struct _mux_config_t const * _k64f_pinmux_config_table[e_K64F_PORT_FUNCTION_COUNT] = {\
	0,/*nulll table for GPIO, need special handling*/
	_mux_config_k64_pwm,
	_mux_config_k64_adc,
	_mux_config_k64_sda,
	_mux_config_k64_scl,
	_mux_config_k64_rmii,
	_mux_config_k64_spi_mosi,
	_mux_config_k64_spi_sclk,
	0,
	0,
};
static uint8_t _k64f_pinmux_config_table_size[e_K64F_PORT_FUNCTION_COUNT] = {\
	0,/*nulll table for GPIO, need special handling*/
	_mux_config_k64_pwm_count,
	_mux_config_k64_adc_count,
	_mux_config_k64_sda_count,
	_mux_config_k64_scl_count,
	_mux_config_k64_rmii_count,
	_mux_config_k64_spi_mosi_count,
	_mux_config_k64_spi_sclk_count,
	0,
	0,
	
};
uint32_t _k64f_get_mux_config(uint8_t k64f_pin_id, enum _port_function_type_t type)
{
	int i;
	uint32_t muxv = _K64F_MUX_INVALID;
	
	struct _mux_config_t const *mux_table;
	if (type >= e_K64F_PORT_FUNCTION_COUNT)
		return muxv;
	else if (type == e_K64F_PORT_FUNCTION_GPIO)
		return _K64F_MUX_GPIO;

	mux_table = _k64f_pinmux_config_table[type];
	for (i=0; i<_k64f_pinmux_config_table_size[type]; i++) {
		if (k64f_pin_id == mux_table[i].pin_id){
			muxv = (uint32_t)mux_table[i].mux_val;
			break;
		}
	}

	return muxv;
}

#endif /*  CONFIG_BOARD_FRDM_K64F */
