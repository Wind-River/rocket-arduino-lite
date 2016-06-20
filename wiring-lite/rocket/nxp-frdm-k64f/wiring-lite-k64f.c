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
 
#ifdef CONFIG_BOARD_FRDM_K64F

#include <zephyr.h>
#include <gpio.h>
#include <pwm.h>
#include <adc.h>

#include <pinmux.h>

#include "pinmux-map-k64f.h"

//copy from drivers/pinmux/pinmux_k64.h
#define K64_PINMUX_GPIO_DIR_INPUT	(0x0 << 20)		/* input GPIO pin */
#define K64_PINMUX_GPIO_DIR_OUTPUT  (0x1 << 20)		/* output GPIO pin */

#define K64_PINMUX_ALT_MASK			(0x7 << 8)
#define K64_PINMUX_ALT_0			(0x0 << 8)
#define K64_PINMUX_ALT_1			(0x1 << 8)
#define K64_PINMUX_ALT_2			(0x2 << 8)
#define K64_PINMUX_ALT_3			(0x3 << 8)
#define K64_PINMUX_ALT_4			(0x4 << 8)
#define K64_PINMUX_ALT_5			(0x5 << 8)
#define K64_PINMUX_ALT_6			(0x6 << 8)
#define K64_PINMUX_ALT_7			K64_PINMUX_ALT_MASK
#define K64_PINMUX_FUNC_GPIO		K64_PINMUX_ALT_1



#include <pwm.h>
/*
#include <spi-k64f.h>
#include <C12832.h>*/


//#include "adc-k64f.h"
//copy from drivers/adc/adc-k64f.h
#define K64F_ADC_RN_COMBINE(byte1, byte2) (uint16_t)(((byte1<<8)&0xff00)|((byte2)&0xff))

#include "Arduino-lite.h"

#ifndef CONFIG_WIRING_LITE_K64F_DEBUG
	#define DBG_PRINT(...) { ; }
#else
	#if defined(CONFIG_STDOUT_CONSOLE)
		#include <stdio.h>
		#define DBG_PRINT printf
	#else
		#include <misc/printk.h>
		#define DBG_PRINT printk
	#endif /* CONFIG_STDOUT_CONSOLE */
#endif /* CONFIG_WIRING_LITE_K64F_DEBUG */


#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#define PRINT           printf
#else
#include <misc/printk.h>
#define PRINT           printk
#endif

#include <string.h>


struct device *pinmux;

struct device *_gpio_ports_k64f[_K64F_PIN_PORT_COUNT];

/***************************************************************************************************
 *
 * Interface implementation
 *
 **************************************************************************************************/
/*
 * gpioOutputSet - set the output value of a pin
 *
 * outputPin: Arduino connector pin number
 * value: 0=set output to low, otherwise set high
 *
 * Note: comments below "gpioXX" refer to the Linux GPIO naming convention
 */


struct _pin_map_by_arduino_t {
	uint8_t		arduino_pin;/*io_mux indexed by arduino pin index*/
	uint8_t		k64f_pin_id; /*port index: 0-4 <==> port A to E,  port pin index: 0~31*/
};



/*k64f pin ID format: bit[7-5]=port number, bit[4-0]=port pin index*/
static struct _pin_map_by_arduino_t _k64f_pin_map_by_arduino[] = 
{
	{IO0,	K64F_PIN_ID(PCR_PORT_C,16)},/*D0*/
	{IO1,	K64F_PIN_ID(PCR_PORT_C,17)},
	{IO2,	K64F_PIN_ID(PCR_PORT_B,9)},
	{IO3,	K64F_PIN_ID(PCR_PORT_A,1)},
	{IO4,	K64F_PIN_ID(PCR_PORT_B,23)},
	{IO5,	K64F_PIN_ID(PCR_PORT_A,2)},
	{IO6,	K64F_PIN_ID(PCR_PORT_C,2)},
	{IO7,	K64F_PIN_ID(PCR_PORT_C,3)},
	{IO8,	K64F_PIN_ID(PCR_PORT_A,0)},
	{IO9,	K64F_PIN_ID(PCR_PORT_C,4)},
	{IO10,	K64F_PIN_ID(PCR_PORT_D,0)},
	{IO11,	K64F_PIN_ID(PCR_PORT_D,2)},
	{IO12,	K64F_PIN_ID(PCR_PORT_D,3)},
	{IO13,	K64F_PIN_ID(PCR_PORT_D,1)},/*D13*/
	{IO14,	K64F_PIN_ID(PCR_PORT_E,25)},
	{IO15,	K64F_PIN_ID(PCR_PORT_E,24)},/*D15*/
	{A0,	K64F_PIN_ID(PCR_PORT_B,2)},/*A0*/
	{A1,	K64F_PIN_ID(PCR_PORT_B,3)},
	{A2,	K64F_PIN_ID(PCR_PORT_B,10)},
	{A3,	K64F_PIN_ID(PCR_PORT_B,11)},
	{A4,	K64F_PIN_ID(PCR_PORT_C,11)},
	{A5,	K64F_PIN_ID(PCR_PORT_C,10)},/*A5*/
	/*extended pins on FRDM K64F */
	{A5+1,  K64F_PIN_ID(PCR_PORT_B,22)},/*red LED on board*/
	{A5+2,  K64F_PIN_ID(PCR_PORT_B,21)},/*blue LED on board*/
	{A5+3,  K64F_PIN_ID(PCR_PORT_E,26)},/*RED LED on board*/
	
	{A5+4,  K64F_PIN_ID(PCR_PORT_C,6)},/*SW2 button on board*/
	{A5+5,  K64F_PIN_ID(PCR_PORT_A,4)},/*SW3 button on board*/
	
};


#define _k64f_pin_map_by_arduino_count (sizeof(_k64f_pin_map_by_arduino)/sizeof(struct _pin_map_by_arduino_t))

static uint8_t _map_from_arduino_id(uint8_t arduino_id)
{
	uint8_t i=0; 
	for (i=0; i<_k64f_pin_map_by_arduino_count; i++){
		if (_k64f_pin_map_by_arduino[i].arduino_pin == arduino_id) {
			return _k64f_pin_map_by_arduino[i].k64f_pin_id;
		}
	}

	return (uint8_t)(-1);
}

int gpioOutputSet (int outputPin, int value)
{
    int rc1 = DEV_FAIL;
	uint8_t k64f_pin_id = _map_from_arduino_id(outputPin);
	int gpio_port_index = K64F_PIN_PORT(k64f_pin_id);
	int gpio_pin_index = K64F_PIN_INDEX(k64f_pin_id);
	struct device * gpio_port = _gpio_ports_k64f[gpio_port_index];

	if ( (32 <= gpio_pin_index) || (_K64F_PIN_PORT_COUNT <= gpio_port_index) || (0 == gpio_port))
		{
			rc1 = DEV_FAIL;
		}
	else 
        {
			gpio_pin_write(gpio_port, gpio_pin_index, value);
			rc1 = DEV_OK;
		
		}

    if (rc1 != DEV_OK)
        {
        PRINT("GPIO write error %d!!\n", rc1);
        }

    return rc1;
}

/*
 * gpioInputGet - get value of a pin
 *
 * inputPin: Arduino connector pin number
 * Returns: 0 if value is low,
 *          1 if value is high
 *
 * Note: comments below "gpioXX" refer to the Linux GPIO naming convention
 */

int gpioInputGet (int inputPin)
{
    uint32_t value;
	int rc1 = DEV_FAIL;
	uint8_t k64f_pin_id = _map_from_arduino_id(inputPin);
	int gpio_port_index = K64F_PIN_PORT(k64f_pin_id);
	int gpio_pin_index = K64F_PIN_INDEX(k64f_pin_id);

	struct device * gpio_port = _gpio_ports_k64f[gpio_port_index];

	if ( (32 <= gpio_pin_index) || (_K64F_PIN_PORT_COUNT <= gpio_port_index) || (0 == gpio_port))
		{
			rc1 = DEV_FAIL;
		}
	else 
        {
			gpio_pin_read(gpio_port, gpio_pin_index, &value);
			rc1 = DEV_OK;
		
		}

	if (rc1 != DEV_OK)
        {
        PRINT("GPIO read error %d!!\n", rc1);
        }

    if (0 == value)
        return LOW;
    else
        return HIGH;
}


void pinMode(uint8_t pin, uint8_t mode) {

	uint8_t k64f_pin_id = _map_from_arduino_id(pin);
	int gpio_port_index = K64F_PIN_PORT(k64f_pin_id);
	int gpio_pin_index = K64F_PIN_INDEX(k64f_pin_id);

	struct device * gpio_port = _gpio_ports_k64f[gpio_port_index];
	
	DBG_PRINT("Initialized pin%d\n", pin);
	/*if ( A0 <= pin)
	{
		DBG_PRINT("pinMode can only work over digital pins\n");
		return;
	}*/
	
	if (  (32 <= gpio_pin_index) || (_K64F_PIN_PORT_COUNT <= gpio_port_index) || (0 == gpio_port))
	{
		//DBG_PRINT("pinMode can only work over digital pins\n");
		return;
	}


    /* set direction  */
    switch (mode) {
	case INPUT_PULLUP:
	case INPUT:
    	DBG_PRINT("Setting mode INPUT\n");
		   
           if (pinmux_pin_set(pinmux, K64F_PIN_ID_2_INDEX(_k64f_pin_map_by_arduino[pin].k64f_pin_id), (K64_PINMUX_FUNC_GPIO|K64_PINMUX_GPIO_DIR_INPUT))) {
    	      DBG_PRINT("failed\n");
           } else {
    	      DBG_PRINT("success\n");
           }
		   
           /*if (gpio_pin_configure(gpio_port, gpio_pin_index, GPIO_DIR_IN)) {
    	      DBG_PRINT("failed\n");
           } else {
    	      DBG_PRINT("success\n");
           }*/
	break;
        case OUTPUT:
    	   DBG_PRINT("Setting mode to OUTPUT...");
		   if(pinmux_pin_set(pinmux, K64F_PIN_ID_2_INDEX(_k64f_pin_map_by_arduino[pin].k64f_pin_id),(K64_PINMUX_FUNC_GPIO|K64_PINMUX_GPIO_DIR_OUTPUT)))
		   {
    	      DBG_PRINT("failed\n");
           } else {
    	      DBG_PRINT("success\n");
           }
		   
           /*if (gpio_pin_configure(gpio_port, gpio_pin_index, GPIO_DIR_OUT)) {
    	      DBG_PRINT("failed\n");
           } else {
    	      DBG_PRINT("success\n");
           }*/
    }

}

void digitalWrite(register uint8_t pin, register uint8_t value)
{
    if ( value == HIGH ){
    	DBG_PRINT("Setting pin to HIGH...");}
    else {
    	DBG_PRINT("Setting direction to LOW...");}
    if (gpioOutputSet(pin, value)) { 
    	DBG_PRINT("Failed\n");
    } else {
    	DBG_PRINT("Success\n");
    }
}

int digitalRead(uint_t pin)
{
	uint8_t val;

	val = gpioInputGet (pin);
        DBG_PRINT("Gpio%d is %d\n", pin, val);

	return val;
}


struct _pwm_map_t {
	uint8_t pin_id;
	//uint8_t ftm;
	/*const */char * ftm_name;
	uint8_t chn;
	struct device *ftm_dev;

};

struct _pwm_map_t _k64f_pwm_map[] = 
{
#if CONFIG_PWM_K64_FTM
#if CONFIG_PWM_K64_FTM_0
	{K64F_PIN_ID(PCR_PORT_A,1),CONFIG_PWM_K64_FTM_0_DEV_NAME,6,0},
	{K64F_PIN_ID(PCR_PORT_A,2),CONFIG_PWM_K64_FTM_0_DEV_NAME,7,0},
	{K64F_PIN_ID(PCR_PORT_C,2),CONFIG_PWM_K64_FTM_0_DEV_NAME,1,0},
	{K64F_PIN_ID(PCR_PORT_C,3),CONFIG_PWM_K64_FTM_0_DEV_NAME,2,0},	
	{K64F_PIN_ID(PCR_PORT_A,0),CONFIG_PWM_K64_FTM_0_DEV_NAME,5,0},
	{K64F_PIN_ID(PCR_PORT_C,4),CONFIG_PWM_K64_FTM_0_DEV_NAME,3,0},
#endif //#if CONFIG_PWM_K64_FTM_0

#if CONFIG_PWM_K64_FTM_3
	{K64F_PIN_ID(PCR_PORT_D,0),CONFIG_PWM_K64_FTM_3_DEV_NAME,0,0},
	{K64F_PIN_ID(PCR_PORT_D,2),CONFIG_PWM_K64_FTM_3_DEV_NAME,2,0},
	{K64F_PIN_ID(PCR_PORT_D,3),CONFIG_PWM_K64_FTM_3_DEV_NAME,3,0},
	{K64F_PIN_ID(PCR_PORT_D,1),CONFIG_PWM_K64_FTM_3_DEV_NAME,1,0},
#endif //#if CONFIG_PWM_K64_FTM_3

/*A4 and A5 support PWM as well, but depreciate them for now*/
/*
	{K64F_PIN_ID(PCR_PORT_C,11),3,7,0},
	{K64F_PIN_ID(PCR_PORT_C,10),3,6,0},
*/
#endif //#if CONFIG_PWM_K64_FTM
};
#define _k64f_pwm_map_size  ((sizeof(_k64f_pwm_map))/(sizeof(struct _pwm_map_t)))

int pwm_k64f_map_get(uint8_t k64f_pin_id, struct device **ftm_dev, uint8_t *ch)
{
	int i;
	if (ftm_dev == NULL || ch == NULL)
		return -2;
	
	*ftm_dev = 0;
	*ch = 0xff;
	
	for (i = 0; i < _k64f_pwm_map_size; i++) {
	    if (k64f_pin_id == _k64f_pwm_map[i].pin_id) {
		    *ch = _k64f_pwm_map[i].chn;
			if (0 == _k64f_pwm_map[i].ftm_dev) {
				_k64f_pwm_map[i].ftm_dev = device_get_binding(_k64f_pwm_map[i].ftm_name);
			}
			*ftm_dev = _k64f_pwm_map[i].ftm_dev;
			break;
		}
    }

	return (0 == *ftm_dev ? -1 :0);
}


/*steps:
1, pinmux change pin's mux to PWM
2, set pwm duty cycle
*/
void analogWrite(uint32_t pin, uint32_t value)
{
	uint8_t k64f_pin_id = _map_from_arduino_id(pin);

	struct device *ftm_dev=NULL;
    uint8_t pwm = 0xFF;
    uint8_t i = 0;

    DBG_PRINT("Initialized pin%d\n", pin);
    if ( (IO3== pin ) || ((IO5 <= pin) && (IO13 >= pin)) ) {
	    DBG_PRINT("GPIO pin %d\n", pin);

	    /* find the PWM mapping */
	    i = pwm_k64f_map_get(k64f_pin_id, &ftm_dev, &pwm);
	    if (i != 0) {
		    DBG_PRINT("%s bad pwm pin %d\n", __FUNCTION__, pin);
		    return ;
	    }
		
		uint32_t muxv = _k64f_get_mux_config(k64f_pin_id, e_K64F_PORT_FUNCTION_PWM);
		if (muxv == _K64F_MUX_INVALID) {
			DBG_PRINT("%s bad PWM pin mapping %d\n", __FUNCTION__, pin);
		}
		else {
			if (pinmux_pin_set(pinmux, K64F_PIN_ID_2_INDEX(k64f_pin_id), (muxv<<8))) {
				DBG_PRINT("%s pinmux setting failed for PWM pin %d\n",__FUNCTION__, pin);
			} else {
				DBG_PRINT("%s pinmux setting success for PWM pin %d\n",__FUNCTION__, pin);
				
				//pwm_pin_set_duty_cycle_by_8_bit_value(ftm_dev,pwm,value);
				pwm_pin_configure(ftm_dev, pwm, 0);
				
				DBG_PRINT("Writing [%d] to PWM pin [%d]\n",value,pwm);
				pwm_pin_set_duty_cycle(ftm_dev, pwm, (100*value)/256);
				//pwm_pin_set_duty_cycle(ftm_dev, pwm, value);
				
			}
		} 
		

    }
    else if ( (A0 <= pin) && (A5 >= pin)) {
	    DBG_PRINT("ADC pin %d\n", pin);
	    DBG_PRINT("   -- NOT SUPPORTED  -- ADC pin %d\n", pin);
    }
    else {
	    DBG_PRINT("INVALID Operation for pin %d\n", pin);
    }
}


struct _adc_map_t {
	uint8_t pin_id;
	//uint8_t ftm;
	/*const */char * adc_name;
	uint8_t chn;
	struct device *adc_dev;

};

struct _adc_map_t _k64f_adc_map[] = 
{
#ifdef CONFIG_ADC_K64F
#ifdef CONFIG_ADC_K64F_0
	{K64F_PIN_ID(PCR_PORT_B,2),	CONFIG_ADC_K64F_0_DEV_NAME,12,0},	/*adc0_se12*/
	{K64F_PIN_ID(PCR_PORT_B,3),	CONFIG_ADC_K64F_0_DEV_NAME,13,0},	/*adc0_se12*/
#endif //#if CONFIG_ADC_K64F_0
#ifdef CONFIG_ADC_K64F_1
	{K64F_PIN_ID(PCR_PORT_B,10),CONFIG_ADC_K64F_1_DEV_NAME,14,0},	/*ADC1_SE14*/
	{K64F_PIN_ID(PCR_PORT_B,11),CONFIG_ADC_K64F_1_DEV_NAME,15,0},	/*ADC1_SE15*/
	{K64F_PIN_ID(PCR_PORT_C,11),CONFIG_ADC_K64F_1_DEV_NAME,7,0},	/*ADC1_SE7b*/
	{K64F_PIN_ID(PCR_PORT_C,10),CONFIG_ADC_K64F_1_DEV_NAME,6,0},	/*ADC1_SE6b*/
#endif //#if CONFIG_ADC_K64F_1
#endif //#if CONFIG_ADC_K64F

};
#define _k64f_adc_map_count  ((sizeof(_k64f_adc_map))/(sizeof(struct _adc_map_t)))

int adc_k64f_map_get(uint8_t k64f_pin_id, struct device **adc_dev, uint8_t *ch)
{
	int i;
	if (adc_dev == NULL || ch == NULL)
		return -2;
	
	*adc_dev = 0;
	*ch = 0xff;
	
	for (i = 0; i < _k64f_adc_map_count; i++) {
	    if (k64f_pin_id == _k64f_adc_map[i].pin_id) {
		    *ch = _k64f_adc_map[i].chn;
			if (0 == _k64f_adc_map[i].adc_dev) {
				_k64f_adc_map[i].adc_dev = device_get_binding(_k64f_adc_map[i].adc_name);
				if (0 == _k64f_adc_map[i].adc_dev) {
					DBG_PRINT("AnalogRead cannot get device %s\n", _k64f_adc_map[i].adc_name);
				}
			}
			*adc_dev = _k64f_adc_map[i].adc_dev;
			break;
		}
    }

	return (0 == *adc_dev ? -1 :0);
}


uint16_t analogRead(uint8_t pin)
{
	uint8_t k64f_pin_id = _map_from_arduino_id(pin);
    int rc;

	struct device *adc_dev=NULL;
    uint8_t adc_chn = 0xFF;

	
    DBG_PRINT("Initialized pin%d\n", pin);
    if ( (A0 <= pin) && (A5 >= pin) ) {
	    DBG_PRINT("Analog pin %d\n", pin);

	    /* find the ADC mapping */
	    rc = adc_k64f_map_get(k64f_pin_id, &adc_dev, &adc_chn);
	    if (rc != 0) {
		    DBG_PRINT("%s bad ADC pin %d/%d\n", __FUNCTION__, pin, k64f_pin_id);
		    return (uint16_t)(-1);
	    }

		uint32_t muxv = _k64f_get_mux_config(k64f_pin_id, e_K64F_PORT_FUNCTION_ADC);
		if (muxv == _K64F_MUX_INVALID) {
			DBG_PRINT("%s bad ADC pin mapping %d\n", __FUNCTION__, pin);
		}
		else {
			if (pinmux_pin_set(pinmux, K64F_PIN_ID_2_INDEX(k64f_pin_id), (muxv<<8))) {
				DBG_PRINT("%s pinmux setting failed for ADC pin %d\n",__FUNCTION__, pin);
			} else {
				DBG_PRINT("%s pinmux setting success for ADC pin %d\n",__FUNCTION__, pin);

				/*sync mode only test*/
				uint8_t buff[256];
				struct adc_seq_entry entry;
				struct adc_seq_table table;
				table.entries=&entry;
				table.num_entries=1;
				entry.channel_id = adc_chn;
				entry.buffer = &buff[0];
				entry.buffer_length = sizeof(buff);
				
				rc = adc_read(adc_dev, &table); 	
				if (0 == rc){
					uint16_t input_temp = K64F_ADC_RN_COMBINE(buff[0],buff[1]);
					/*mapping from 0~65535 to 0~1023*/
					return map(input_temp, 0, 65535, 0, 1023);
				}

			}
		}


    }
    else {
	    DBG_PRINT("INVALID Operation for pin %d\n", pin);
		//return (uint16_t)(-1);
    }

    //return val;
    return (uint16_t)(-1);
}

//#include <stdio.h>
void hardware_init() 
{

#if CONFIG_PINMUX_K64
    /* get the device bindings */
	pinmux = device_get_binding(PINMUX_NAME);
#endif //#if CONFIG_PINMUX_K64
	
#if CONFIG_GPIO_K64
	memset(_gpio_ports_k64f,0, sizeof(_gpio_ports_k64f));

    /* On-chip Quark GPIO */
    _gpio_ports_k64f[0] = device_get_binding(CONFIG_PINMUX_K64_GPIO_A_NAME);            // GPIO Port A
	_gpio_ports_k64f[1] = device_get_binding(CONFIG_PINMUX_K64_GPIO_B_NAME);			 // GPIO Port B
	_gpio_ports_k64f[2] = device_get_binding(CONFIG_PINMUX_K64_GPIO_C_NAME);			 // GPIO Port C
	_gpio_ports_k64f[3] = device_get_binding(CONFIG_PINMUX_K64_GPIO_D_NAME);			 // GPIO Port D
	_gpio_ports_k64f[4] = device_get_binding(CONFIG_PINMUX_K64_GPIO_E_NAME);			 // GPIO Port E

#endif


#if CONFIG_I2C_K64F_0
	//set pinmux for Arduino I2C header
	uint8_t k64f_pin_id = _map_from_arduino_id(SDA);

	uint32_t muxv = _k64f_get_mux_config(k64f_pin_id, e_K64F_PORT_FUNCTION_SDA);
	if (muxv == _K64F_MUX_INVALID) {
		DBG_PRINT("%s bad I2C SDA pin mapping %d\n", __FUNCTION__, pin);
	}
	else {
		if (pinmux_pin_set(pinmux, K64F_PIN_ID_2_INDEX(k64f_pin_id), (muxv<<8))) {
			DBG_PRINT("%s pinmux setting failed for I2C SDA pin %d\n",__FUNCTION__, pin);
		} else {
			DBG_PRINT("%s pinmux setting success for I2C SDA pin %d\n",__FUNCTION__, pin);
		}
	}

    
    k64f_pin_id = _map_from_arduino_id(SCL);
	muxv = _k64f_get_mux_config(k64f_pin_id, e_K64F_PORT_FUNCTION_SCL);
	if (muxv == _K64F_MUX_INVALID) {
		DBG_PRINT("%s bad I2C SCL pin mapping %d\n", __FUNCTION__, pin);
	}
	else {
		if (pinmux_pin_set(pinmux, K64F_PIN_ID_2_INDEX(k64f_pin_id), (muxv<<8))) {
			DBG_PRINT("%s pinmux setting failed for I2C SCL pin %d\n",__FUNCTION__, pin);
		} else {
			DBG_PRINT("%s pinmux setting success for I2C SCL pin %d\n",__FUNCTION__, pin);
		}
	}
	
#endif //#if CONFIG_I2C_K64F_0

    return;
}

#endif /*  CONFIG_BOARD_FRDM_K64F */
