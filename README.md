## Wind River Rocket Arduino-lite API 
# Arduino-lite Library for Rocket  
***  

## Introduction

The **Wind River Rocket Arduino-lite** API (**Arduino-lite**) provides a simple and familiar programming model to 
modify, create, debug, and run example code on devices using the Wind River [**Helix App Cloud**](https://app.cloud.windriver.com/). 
With a few simple functions, the devices can be programmed to register
and communicate with other Cloud services.
  
This module is meant to be a simple system that uses the [**Wiring**](http://wiring.org.co/) paradigm and implements a subset of the **Wiring/Arduino**
APIs, some **Arduino** compatible libraries, networking, and IoT protocols.  It is provided under the [**Apache 2.0 license**](http://www.apache.org/licenses/LICENSE-2.0).    

## Supported Hardware
* [Intel Galileo Gen 2](http://www.intel.com/content/www/us/en/embedded/products/galileo/galileo-overview.html) (digital read, digital write, analog read, PWM out, I2C, Ethernet )
* [NXP FRDM-K64F](http://www.nxp.com/products/software-and-tools/hardware-development-tools/freedom-development-boards/freedom-development-platform-for-kinetis-k64-k63-and-k24-mcus:FRDM-K64F) (digital read, digital write, analog read, PWM out, Ethernet)
* The Sketch examples that use this library were tested with Seeed Studio [Grove - Starter Kit for Arduino](http://www.seeedstudio.com/depot/Grove-Starter-Kit-for-Arduino-p-1855.html)

## Supported Software
* Wind River [Helix Appj Cloud](https://app.cloud.windriver.com/) Development Environment(Intel Galileo Gen 2 SDK and NXP FRDM-K64F SDK) 
  * Editing and browsing source code
  * Compiling and running sketches on devices
  * Setting breakpoints and stepping through code
  * Watching call stacks and variables
  * Other features
* The rocket-arduino-lite library which is built upon [Zephyr](https://www.zephyrproject.org/), [Light Weight IP](http://savannah.nongnu.org/projects/lwip/), and [Paho Embedded-C client](http://www.eclipse.org/paho/clients/c/embedded/)
* Several example Sketches for
  * Using sensors
  * Basic networking (DHCP, TCP/IP, DNS, MQTT)
  * Connectivity to the [IBM Watson IoT Platform](http://www.ibm.com/internet-of-things/trial.html?cm_mmc=search-gsn-_-branded-watson-iot-search-_-watson%20iot-phrase-_-NA-iot-mkt-ow)    

## General program structure

The structure of programs within the Arduino-Lite environment is similar to that of 
Arduino sketches with some exceptions:
* The main library, rocket-arduino-lite, was developed independently and has its own structure.
* The rocket-arduino-lite library was developed under the Apache 2.0 license.
* Examples are built on Sketches that use  *setup()* and *loop()* functions contained
in a C++ file (.cpp).  The sketch is located in the application/src directory of a
 project.
* The main Wiring-lite APIs are defined in Arduino-lite.h  
Though optional, it is encouraged to include Arduino-lite.h in your sketch.  This
will make the Helix App Cloud IDE aware of the included functions.
* The basic Wiring APIs are contained in the arduino-lite/wiring-lite hierarchy.
* Other C++ Libraries are available under the arduino-lite/libraries hierarchy.
* rocket-arduino-lite depends on other third party software that is pulled into the
project at project creation time.  
* Example sketches that use this library have the repositories prefixed with **rocket-arduino-lite-sketch-**.  


## Limitations

The Arduino-Lite API provides a basic subset of Arduino API calls within the Rocket environment.   
A list of the supported Arduino calls, limitations, or enhancements is listed below.
*** 
***


 
## Wiring API 
```
#include "Arduino-lite.h`
```

Most of the functionality in the [Arduino Language Reference](https://www.arduino.cc/en/Reference/HomePage) is supported.
The following lists the compatibility with the Arduino Language Reference.

### 
**Structure**
* **Control Structures:** ***Fully*** Supported
* **Arithmetic Operators:** ***Fully*** supported
* **Comparison Operators:** ***Fully*** supported
* **Boolean Operators:** ***Fully*** supported
* **Pointer Access Operators:** ***Fully*** supported
* **Bitwise Operators:** ***Fully*** supported
* **Compound Operators:** ***Supported for built-in types only***  
No classes implement **operator** methods.    
  
**Variables**
* **Constants:** ***Partially*** supported  
**INPUT_PULLUP**, **LED_BUILTIN**, **integer constants**, and **floating point constants** not implemented
* **Data Types:** ***Mostly*** supported  
**String** not supported
* **Conversion:** ***Fully*** supported  
Based on C++ cast rules
* **Variable Scope & Qualifiers:** ***Fully*** supported  
Based on C++ scoping and **const** rules
* **Utilities:** ***Partially*** supported  
**PROGMEM** not supported  
    
**Functions**
* **Digital I/O:** ***Mostly*** supported  
**INPUT_PULLUP** not implemented 
* **Analog I/O:** ***Mostly*** supported  
**analogReference** not implemented
* **Due & Zero Only:** ***Not*** supported
* **Advanced I/O:** ***Not*** supported
* **Time:** ***Fully*** supported
* **Math:** ***Fully supported with noted exceptions***  
Math constants not implemented  
Other math functions not listed may be available but have not been tested
* **Trigonometry:** ***Fully supported with noted exceptions***  
Math constants not implemented  
Other trigonometry functions not listed may be available but have not been tested
* **Characters:** ***Not*** implemented
* **Random Numbers::** **Not** implemented  
**srand** and **rand** are available with the math library but have not been tested
* **Bits and Bytes:** ***Not*** implemented
* **External Interrupts:** ***Fully*** supported on pins **3** and **4** with the caveat that 
interrupts are simulated by polling the pins from a higher priority task. Note that pin **2** is not supported
* **Interrupts:**  ***Fully*** supported API as noted  
The functions only apply to the simulated External Interrupts and have no effect on processor interrupts
* **Communication**: ***Very Basic*** support for printing to console of Helix App Cloud IDE 
***Stream*** not implemented  
A singleton ***Serial*** variable exists  
***Serial*** does not control the serial pins.  Only Serial.print and Serial.println are implemented as way of printing to standard out (which appears on the Helix App Cloud build tab)
* **USB:** ***Not*** supported
***
***



## Additional Libraries

### Ethernet Library (Intel Galileo Gen2 SDK and NXP FRDM-K64F SDK)
```
include "Ethernet.h"
```
The Ethernet library implements basic versions of Ethernet and Ethernet Client classes.  It uses
the Light Weight IP stack in single threaded (**NO_SYS**) mode.

**Ethernet** object (derived from EthernetClass)
* Only one object, **Ethernet** (created as static constructor) permitted
* Sends and receives packets (using Service() member function)
* Retrieves MAC address from NIC.  Setting of the MAC address is ignored for all functions.  
* Supports DHCP and static addresses

Only the following **Arduino** functions are supported:  
* `void begin(byte* mac)`   
Note: that MAC address is read from the driver so the **mac** parameter is ignored.  
* `void begin(byte* mac, byte* ip)`  
Note that **ip** is an array of 4 bytes -- not an IPAddress. **mac** is ignored.  
* `void begin(byte* mac, byte* ip, byte* dns)`
Note that **ip** and **dns** are arrays of 4 bytes. **mac** is ignored.  
* `void begin(byte* mac, byte* ip, byte* dns, byte* gateway)`   
Note that **ip**, **dns**, and **gateway** are arrays of 4 bytes. **mac** is ignored.    
* `void begin(byte* mac, byte* ip, byte* dns, byte* gateway, byte* subnet)`   
Note that **ip**, **dns**, **gateway**, and **subnet** are arrays of 4 bytes. **mac** is ignored.    
* `bool connected()`   


The following public members functions are also supported:  
* `void service()`  
* `void begin()` 
* `void getMacAddress(byte address[])`

**Ethernet Client** object (derived from EthernetClient)
* Only one object, **Ethernet_client** (created as static constructor)
* Connects to a port on a remote server
* Supports raw TCP/IP only (no sockets)
* Supports destination IP address and resolving hosts with DNS

Only the following **Arduino** functions are supported:  
* `int connect(char* ip, int port)`
* `boolean connected()`
* `void disconnect()`
* `int write(char ch)`
* `int write(char* buffer, int length)`
* `char read()`
* `int available()`
***



### MqttClient Library (Intel Galileo Gen2 SDK and NXP FRDM-K64F SDK)
```
#include "MqttClient.h"
```
The MqttClient library implements basic Mqtt Client functionality.  It uses
both the Ethernet and Ethernet_client objects to bring the network up and 
connect to the port on a remote MQTT broker.  The MQTT protocol is implemented
using the Paho Embedded-C client. The API provides a subset of the [PubSubClient](http://pubsubclient.knolleary.net/api.html) calls.

**Mqtt_client** object 
* Only one object, **Mqtt_Client** (created as static constructor) permitted
* Connects to the remote MQTT broker, publishes and subscribes topics.

Only the following public member functions are supported:  
* `boolean connect(char* id)`
* `boolean connect(char* id, char* user, char* pass)`
* `boolean connect(char* id, char* willTopic, uint8_t willQos, boolean willRetain, char* willMessage)`
* `boolean connect(char* id, char* user, char* pass, char* willTopic, uint8_t willQos, boolean willRetain, char* willMessage)`
* `void setCallback(void (*)(char*, byte*, unsigned int))`
* `boolean subscribe(const char* topic, uint8_t qos = 0)`
* `boolean publish(const char* topic,char* payload, uint8_t qos = 0)`
* `void setServer(char * ip, uint16_t port)`
* `void loop()`
* `boolean connected()`
* `void begin(byte* mac)`   


```
#include "Serial.h"
#include "Ethernet.h" 
#include "MqttClient.h"

const int pubWaitCount = 20;
int pubCount = 0;
uint8_t qos = 1;

void MQTTMessageReceived(char* topic, byte* payload, unsigned int length) {
    Serial.print("Rocket received message  [");
    Serial.print(topic);
    Serial.print("] ");
  
    for (int i=0;i<length;i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();  
}

void setup() {
    Serial.println("Bringing up Ethernet interface with DHCP address");
   
   // Bring up the inteface with a DHCPed address (MACC address param ignored; will use real MAC address)
    Ethernet.begin(); // Bring up interface with H/W MAC address and DHCP Ethernet Address
  
    MQTT_Client.setCallback(MQTTMessageReceived);
    MQTT_Client.setServer("iot.eclipse.org", 1883);
}

void loop() {    
    if (!MQTT_Client.connected()) {    
        byte mac[] = { 1, 2, 3, 4, 5, 6 };
        char id[] = "AABBCCDDEEFF"; // will be replaced by MAC address
         Ethernet.getMacAddress(mac) ;   
        sprintf(id,"%X%X%X%X%X%X%X%X%X%X%X%X", (mac[0] >> 4), (mac[0] & 0xf), (mac[1] >> 4), (mac[1] & 0xf), (mac[2] >> 4), (mac[2] & 0xf), (mac[3] >> 4), (mac[3] & 0xf), (mac[4] >> 4), (mac[4] & 0xf), (mac[5] >> 4), (mac[5] & 0xf));

      if (MQTT_Client.connect(id)) {
          if (!MQTT_Client.subscribe("rocket/greeting", qos)) {
             Serial.println("Subscribing failed");
          }
      } else {
        Serial.println("Connection failed");
      }
  } else {   
        if (pubCount < pubWaitCount) {
            pubCount++;
        } else {
            pubCount = 0;
            if (!MQTT_Client.publish("rocket/greeting", "hello from rocket", qos)){
                Serial.println("Publishing Failed");
            }      
        }
        MQTT_Client.loop();
  }
  delay(100);
}
```
***




### BluemixClient Library (Intel Galileo Gen2 SDK and NXP FRDM-K64F SDK)
```
#include "BluemixRocketClient.h"
```
The BluemixClient library provides simple APIs to communicate with the IBM Watson IoT Platform in Quickstart and Registered Mode. 
For Rocket there are two classes: BluemixRocketClient and DataPoint class.

**DataPoint** class
The following public member functions are supported:  
* `Datapoint(const char name[], float* data)`  
Constructor for a datapoint that references a float variable that is being monitored
* `Datapoint(const char name[], int* data)`     
Constructor for a datapoint that references an int variable that is being monitored
* `Datapoint(const char name[], char* data)`  
Constructor for a datapoint that references a string variable that is being monitored
* `show()`


**BluemixRocketlient** class  
The following public member functions are supported:  
* `BluemixRocketClientClass()`  
Contructor for Quickstart mode. Will connect with the MAC adddress as unique ID.  Only publishing data is supported
in Quickstart mode.
* `BluemixRocketClientClass(char* type, char* org, char* token, void (*callback)(char*, byte*, unsigned int))`  
Constructor for Registered mode.  Will connect with the MAC address as unique ID.  Requires a callback for receiving
messages from Bluemix and requires the device type, **type**, organization, **org**, and authentication token,
**token** obtained from the setup in the IBM Bluemix web site.   
* `boolean connected()`
* `boolean connect()`
* `boolean publish(Datapoint* d[], int num)`

**Quickstart Mode**  uses the following constructor and has no callback function  
```
BluemixRocketClientClass BluemixClient;
```

```
#include "Serial.h"
#include "BluemixRocketClient.h"

#define PUB_WAIT_COUNT 20


// datapoints to sent to Bluemix

float temperature;
Datapoint temp("temp", &temperature);

#define IOT_OS "Wind River Rocket with Paho MQTT Embedded-C agent on top of Light Weight IP stack"
Datapoint os("iot os", IOT_OS);

#define BOARD "Intel Galileo Gen 2 with Grove Sensor Shield"
Datapoint board("board", BOARD);

Datapoint* datapoints[] = { &os, &board, &temp, };

int temp_input = A0;
int pubCount = 0;
const int B = 3975;
int input = 0;
float resistance;

BluemixRocketClientClass BluemixClient;

void setup() {
    pinMode(temp_input, INPUT);  
}

void loop() {    
    if (!BluemixClient.connected()) {
        if (!BluemixClient.connect()) {
            Serial.println("Connection Failed");
        } else {
            Serial.println("Connected");
        }
    } else {
        if (pubCount < PUB_WAIT_COUNT) {
             pubCount++;
        } else {
            pubCount = 0;
            
            input = analogRead(temp_input);
            resistance = (10230000 - (input * 10000)) / input;
            temperature = 1 / ( log(resistance/10000) / B + (1/298.15)) - 273.15;
            
            if (BluemixClient.publish(datapoints, 3)) {
                Serial.println("Publishing Failed");
            }
        }
    }
  delay(100);
}
```

**Registered Mode** uses the following constructor and has a callback function  
```
#define DEVICE_TYPE "niheer"
#define ORGANIZATION_ID "gqh94c"
#define TOKEN "@uBpMwz&b?eXRb0+5D"  
BluemixRocketClientClass BluemixClient(DEVICE_TYPE, ORGANIZATION_ID, TOKEN, MQTTMessageReceived);
```
```
#include "Serial.h"
#include "BluemixRocketClient.h"


#define DEVICE_TYPE "niheer"
#define ORGANIZATION_ID "gqh94c"
#define TOKEN "@uBpMwz&b?eXRb0+5D"  
    

#define PUB_WAIT_COUNT 20


// datapoints to sent to Bluemix

float temperature;
Datapoint temp("temp", &temperature);

#define IOT_OS "Wind River Rocket with Paho MQTT Embedded-C agent on top of Light Weight IP stack"
Datapoint os("iot os", IOT_OS);

#define BOARD "Intel Galileo Gen 2 with Grove Sensor Shield"
Datapoint board("board", BOARD);

Datapoint* datapoints[] = { &os, &board, &temp, };

int temp_input = A0;
int pubCount = 0;
const int B = 3975;
int input = 0;
float resistance;


void MQTTMessageReceived(char* topic, byte* payload, unsigned int length) {
    Serial.print("Rocket received message  [");
    Serial.print(topic);
    Serial.print("] ");
  
    for (int i=0;i<length;i++) {
        Serial.print((char)payload[i]);
    }
  Serial.println(); 
}

BluemixRocketClientClass BluemixClient(DEVICE_TYPE, ORGANIZATION_ID, TOKEN, MQTTMessageReceived);


void setup() {
    pinMode(temp_input, INPUT);  
}

void loop() {    
    if (!BluemixClient.connected()) {
        if (!BluemixClient.connect()) {
            Serial.println("Connection Failed");
        } else {
            Serial.println("Connected");
        }
    } else {
        if (pubCount < PUB_WAIT_COUNT) {
             pubCount++;
        } else {
            pubCount = 0;
            
            input = analogRead(temp_input);
            resistance = (10230000 - (input * 10000)) / input;
            temperature = 1 / ( log(resistance/10000) / B + (1/298.15)) - 273.15;
            
            if (BluemixClient.publish(datapoints, 3)) {
                Serial.println("Publishing Failed");
            }
  `      }
    }
  delay(100);
}
```
***
## Adding Data Points
Three Datapoints (that send data to the IBM Watson IoT Platform) are used in the example Sketch and are defined as follows:
```
// datapoints to sent to Bluemix

float temperature;
Datapoint temp("temp", &temperature);

#define IOT_OS "Wind River Rocket with Paho MQTT Embedded-C agent on top of Light Weight IP stack"
Datapoint os("iot os", IOT_OS);

#ifdef CONFIG_BOARD_GALILEO
#define BOARD "Intel Galileo Gen 2 with Grove Sensor Shield"
#endif // CONFIG_BOARD_GALILEO

#ifdef CONFIG_BOARD_FRDM_K64F
#define BOARD "NXP FRDM-K64F with Grove Sensor Shield"
#endif // CONFIG_BOARD_FRDM_K64F

Datapoint board("board", BOARD);

Datapoint* datapoints[] = { &os, &board, &temp };
```

The datapoint array and the number of datapoints is passed to the publish function as follows:
```
    if (BluemixClient.publish(datapoints, 3)) {
```

### Example of adding new datapoint to monitor **buttonState**:
```
int ButtonState;
DataPoint button("button", &buttonState);
```

```
Datapoint* datapoints[] = { &os, &board, &temp, &button };
```
```
    if (BluemixClient.publish(datapoints, 4)) {
```


### Servo Library (Intel Galileo Gen2 SDK and NXP FRDM-K64F SDK)
```
#include "Servo.h"
```
The Servo library is a fully supported implementation of the **Arduino** Servo library
which controls servo motors.

The following functions are supported:
* `Servo()`
* `void attach(int pin)`
* `void attach(int pin, int min, int max)`
* `void detach(void)`
* `bool attached(void)`
* `int read(void)`
* `void write(int angle)`
* `void writeMicroseconds(int uS)`
***
```
/*
 * rocket-arduino-lite-application-servo-motor.cpp
 * This example makes use of Arduino servo library to control an RC servo motor.
 * It shows an actuator can be controlled by user specified position or the
 * value of the rotary potentiometer.
 */

#include "Servo.h"

#define SERVO_PIN                 5   /* connector pin is used as servo pin */
#define ROTARY_ANGLE_SENSOR_PIN   A0  /* analog pin used to connect the potentiometer */
#define DELAYTIME                 15  /* delay time for the servo to reach the position */
#define ACTION_DELAY            1500  /* delay time between each action */

int potVal;
int angle;
Servo myservo;

void setup() {
    /* set up servo to attach SERVO_PIN */
    myservo.attach(SERVO_PIN);

    /* wait a while */
    delay(ACTION_DELAY);

    /* set up A0 potentiometer as input source */
    pinMode(ROTARY_ANGLE_SENSOR_PIN, INPUT);

    /* motor rotates from 0 to 180 degrees in steps of one degree */
    for (angle = MIN_ANGLE; angle <= MAX_ANGLE; angle++) {
        myservo.write(angle);
        delay(DELAYTIME);
    }
    delay(ACTION_DELAY);

    /* motor rotates from 180 to 0 degrees in steps of one degree */
    for (angle = MAX_ANGLE; angle >= MIN_ANGLE; angle--) {
        myservo.write(angle);
        delay(DELAYTIME);
    }
    delay(ACTION_DELAY);
}

void loop() {
    /* read the value of the potentiometer */
    potVal = analogRead(ROTARY_ANGLE_SENSOR_PIN);

    /* remap the numbers from 4096 to 180 degree */
    angle = map(potVal, 0, 4096, MIN_ANGLE, MAX_ANGLE);
    printf("potentiometer = %d, angle: = %d\n", potVal, angle);

    /* sets the servo position */
    myservo.write(angle);
	delay(DELAYTIME);
}
```


### Wire Library (Intel Galileo Gen2 SDK only)
```
#include "Wire.h"
```
The wire `library is a fully supported implementation of the **Arduino** Servo Wire
which controls I2C devices.

The following functions are supported:
* `Wire()`
* `void begin(void)`
* `void begin(uint8_t)`
* `void begin(int)`
* `uint8_t requestFrom(uint8_t, uint8_t)`
* `int requestFrom(int, int)`
* `uint8_t requestFrom(uint8_t, uint8_t, bool)`
* `int requestFrom(int, int, bool)`
* `void beginTransmission(uint8_t)`
* `void beginTransmission(int)`
* `int endTransmission(void)`
* `int endTransmission(bool)`
* `uint8_t write(byte)`
* `uint8_t write(const char *)`
* `uint8_t write(uint8_t *, int)`
* `uint8_t available(void)`
* `uint8_t read(void)`
* `void onReceive(void(*)(int))`
* `void onRequest(void(*)(void))`

```
#include "hd44780Lcd.h"
#include "groveLcd_Utils.h"
#include "Wire.h"
#include <stdlib.h>     /* strtol */
#include <string.h>

/* specify which Arduino connector pin is used as input */
#define INPUT_PIN       3
#define DELAYTIME       200
#define LCD_ROW         2
#define LCD_COLUMN      16
#define LCD_MESSAGE1    "We Will Rock U!"

uint8_t lcd_flag         = 0;
bool    flag             = false;
int     inputPinValue    = 1;
int     oldInputPinValue = 0;
const int lcd_addr = LCD_ADDR;      /* #define LCD_ADDR 0x003e */
const int rgb_addr = RGB_ADDR;      /* #define RGB_ADDR 0x0062 */

Wire mywire;

void setup()
{
    mywire.begin();

    /* set up the LCD's number of columns and rows: */
    groveLcd_Begin(LCD_COLUMN, LCD_ROW);

    /* set up D3 button as input source */
    pinMode(INPUT_PIN, INPUT);

    /* Print a message to the LCD. */
    groveLcd_Clear();
    groveLcd_setRGB(0, 100, 200);
    groveLcd_Print(LCD_MESSAGE1);
    delay(1000);
}

void loop()
{
    /* the loop function runs over and over again forever */
    oldInputPinValue = inputPinValue;
    inputPinValue = digitalRead(INPUT_PIN);

    /* Toggle to the cursor on/off if the user pressed the button */
    if (oldInputPinValue != inputPinValue) {
        PRINT("Button on D3 = %d\n", inputPinValue);
        if (inputPinValue == 1)
            flag = !flag;
    }

    if (!flag) {
        /* Set blue backlight */
        groveLcd_setRGB(0, 100, 200);

        /* Turn on the Display */
        groveLcd_Display();
    } else {
        /* Set green backlight */
        groveLcd_setRGB(0, 255, 30);

        /* Turn off the Display */
        groveLcd_noDisplay();
    }

    /* wait a while */
    delay(DELAYTIME);
}
```
### GroveLcd Library (Intel Galileo Gen2 SDK only)
```
#include "GroveLcd.h"
```
The GroveLcd library supports the Grove LCD I2C device. 

The following functions are supported:
* `GroveLcd()`
* `void begin(int, int)`
* `void begin(uint8_t, uint8_t)`
* `void home(void)`
* `void clear(void)`
* `void print(char *)`
* `void setRGB(int, int, int)`
* `void setRGB(uint8_t, uint8_t, uint8_t)`
* `void noDisplay(void)`
* `void display(void)`
* `void noCursor(void)`
* `void cursor(void)`
* `void noBlink(void)`
* `void blink(void)`
* `void scrollDisplayLeft(void)`
* `void scrollDisplayRight(void)`
* `void leftToRight(void)`
* `void rightToLeft(void)`
* `void noAutoscroll(void)`
* `void autoscroll(void)`

```
include "GroveLcd.h"
#include <stdlib.h>
#include <string.h>

/* specify which Arduino connector pin is used as input */
#define INPUT_PIN       3
#define DELAYTIME       200
#define LCD_ROW         2
#define LCD_COLUMN      16
#define LCD_MESSAGE     "We Will Rock U!"

bool    flag             = false;
int     inputPinValue    = 1;
int     oldInputPinValue = 0;

GroveLcd lcd;

void setup()
{
    /* set up the LCD's number of columns and rows: */
    lcd.begin(LCD_COLUMN, LCD_ROW);

    /* set up D3 button as input source */
    pinMode(INPUT_PIN, INPUT);

    /* Print a message to the LCD. */
    lcd.clear();
    lcd.setRGB(0, 100, 200);
    lcd.print(LCD_MESSAGE);
    delay(1000);
}

void loop()
{
    /* the loop function runs over and over again forever */
    oldInputPinValue = inputPinValue;
    inputPinValue = digitalRead(INPUT_PIN);

    /* Toggle to the cursor on/off if the user pressed the button */
    if (oldInputPinValue != inputPinValue) {
        PRINT("Button on D3 = %d\n", inputPinValue);
        if (inputPinValue == 1)
            flag = !flag;
    }

    if (!flag) {
        /* Set blue backlight */
        lcd.setRGB(0, 100, 200);

        /* Turn on the Display */
        lcd.display();
    } else {
        /* Set green backlight */
        lcd.setRGB(0, 255, 30);

        /* Turn off the Display */
        lcd.noDisplay();
    }

    /* wait a while */
    delay(DELAYTIME);
}
```
If you have any questions about the library or have any enhancement requests we encourage you to please join the Rocket Community forum and post your feedback.

