# PetFeeder
- [PetFeeder](#petfeeder)
  * [- Project Idea](#--project-idea)
  * [- Features](#--features)
  * [- Hardware Requirements](#--hardware-requirements)
  * [- Software Requirements](#--software-requirements)
  * [Stm32cube configurations:](#stm32cube-configurations-)
  * [- More hardware details](#--more-hardware-details)
  * [- System Design 1](#--system-design-1)
  * [- System Design 2](#--system-design-2)
  * [- Connections 1](#--connections-1)
  * [- Connections 2](#--connections-2)
  * [- Final Connections](#--final-connections)
  * [- Finial Product](#--finial-product)
  * [- Demo](#--demo)
  * [- References](#--references)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>

## - Project Idea
![alt text](https://raw.githubusercontent.com/marwanH1998/PetFeeder/main/pictures/WhatsApp%20Image%202021-11-17%20at%206.51.38%20PM.jpeg)
The automaic pet feeder, feeding your pet has never been easier!
An automated pet feeder for people who are outside the house for long periods of time and need help feeding their pet.
Will be integrated with Telegram to send notificattions to the owner for water levels and food supply.
Water will be refilled when empty and food will be dispensed at set times of the day.
## - Features
- Telegram Notifications to refill water and food tanks.
- Automated Water Refill.
- Automated Food Refill.
- Set Times for dispensing food automatically
## - Hardware Requirements
- STM32  (Used for all the logic/sensors/actuators with free RTOS and interrupts)
- ESP32  (Used as a wifi modile, to communicate between stm32 and telegram)
- Jumper Wires 
- 6V Battery
- Servo Motor (to open and close the food tank)
- Weight Sensor (to weight the amount of food and amount of water in the plates)
- Ultrasonic Sensor (to measure the available food and water in the tank)
- Water Valve Control (to control water flow)
- Real Time Clock Module (to open the servo motor at specific times)
- HX711 ADC (needed for signal amplification for the load cells (weight)
- 12V Relay (used as a switch for the water valve control)

## - Software Requirements

- Keil uVision
- Stm32CubeMX
- TeraTerm
- Arduino IDE
- Telegram

## Stm32cube configurations:
![alt text](https://raw.githubusercontent.com/marwanH1998/PetFeeder/main/pictures/WhatsApp%20Image%202021-12-03%20at%205.28.52%20PM.jpeg)

- Pins Used STM to ESP32:
```bash
STM32     ESP32
RX (PB7) ->  TX2  
TX (PA9) ->  RX2
```
- Pins Used STM to RTC:
```bash
STM32     RTC
5V   -> VDD
GND  -> GND
PA7  -> SCL
PB4  -> SDA
```
- Pins Used STM to HX711:
```bash
STM32     HX711
PA5  ->  DT
PA4  -> SCK
5V   -> VCC
GND  -> GND

- Pins Used STM to second HX711:
```bash
STM32     HX711
PB1  ->  DT
PB0  -> SCK
5V   -> VCC
GND  -> GND
```
```

- Pins Used STM to SERVO MOTOR:
```bash
STM32     SERVO
PA0   -> PWM
5V    -> 5V
GND   -> GND
```

- Pins Used STM to RELAY:
```bash
STM32     RELAY
PB3     -> IN
5V   -> VDD
GND  -> GND
``````
- Pins Used RELAY to Water Valve:
```bash
RELAY     Water Valve
Normally Closed   -> 12v

- Pins Used RELAY to 12V power adapter:
```bash
RELAY     Power Adapter
COMM  -> 12v
```

```
- Pins Used Weight Sensor to HX711:
```bash
Weight Sensor     HX711
RED            -> E+     
BLACK          -> E-
WHITE          -> A-
GREEN          -> A+
```
- Pins Used STM to Ultra Sonic Sensor (USS):
```bash
STM32     USS 
PB3     -> ECHO
PA8     -> TRIG
5V   -> VCC
GND  -> GND
```
- Pins Used STM to Second Ultra Sonic Sensor (USS):
```bash
STM32     USS 
PB6   -> ECHO
PA10     -> TRIG
5V   -> VCC
GND  -> GND
```
## - More hardware details

-STM32
    We did not buy this MCU, it was already provided by the course lab. we used this MCU as shown in the system design below.

***

-ESP32
    We did buy this MCU from [Link to buy](https://ram-e-shop.com/product/kit-esp32-esp32s/). we used only its TX2 and RX2 to communicate with the STM32, and then send from it to telegram. For telegram, we used this library [library link](https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot)

***
-Servo Motor
    We did not buy the servo motor. It was provided by the workshop in the department and its picture and more details can be found [here](https://servodatabase.com/servo/hitec/hs-300)
***
-Weight Sensor (load cell)
    We did buy the weight sensor and it works with HX711 chip. The HX711 is a 24-bit analog to digital and signal conditioning module. It is designed specifically for weight scales, weight sensor and industrial control applications to interface directly with a bridge sensor. The module is based on HX711 chip. In most Load cells and weight sensors the output range of a strain gauge is very small and thus the signal needs to be amplified before processing to prevent introduction of errors. This module is used for amplifying the weight sensor and converting its analog sensor to digital one, therefore increasing the measurement accuracy.
[HX711 buy link](https://ram-e-shop.com/product/kit-hx711-adc/)
[load cell buy link](https://ram-e-shop.com/product/kit-load-cell-20kg/)
***
-Ultrasonic Sensor
    This was provided by the lab of the course.
***
-Water Valve Control and 5V DC Relay Module.

   We did buy the water valve solenoid, however, it had a problem which is the minimum pressure to work was 0.2 MPa which required a lot of water pressure to operate. Hence, we ordered a new one from amazon which its minimum pressure should be lower. The problem was that those specifications were not mentioned on the description of the product. We also used a 12v dc adapter to feed the relay module with 12v through the adapter's vdd.

[water valve solenoid link](https://ram-e-shop.com/product/solenoid-12vdc-i/)
[Relay module link](https://ram-e-shop.com/product/kit-m2-1relay/)
***





## - System Design 1
![alt text](https://raw.githubusercontent.com/marwanH1998/PetFeeder/main/pictures/Untitled%20Diagram.jpg)

## - System Design 2
![alt text](https://raw.githubusercontent.com/marwanH1998/PetFeeder/main/pictures/Embedded.drawio.png)

## - Connections 1
![alt text](https://raw.githubusercontent.com/marwanH1998/PetFeeder/main/pictures/connection_1.jpeg)
## - Connections 2
![alt text](https://raw.githubusercontent.com/marwanH1998/PetFeeder/main/pictures/WhatsApp%20Image%202021-12-03%20at%206.06.45%20PM.jpeg)
## - Final Connections 
![alt text](https://raw.githubusercontent.com/marwanH1998/PetFeeder/main/pictures/WhatsApp%20Image%202021-12-12%20at%208.43.32%20PM%20(2).jpeg)
## - Finial Product
![alt text](https://raw.githubusercontent.com/marwanH1998/PetFeeder/main/pictures/WhatsApp%20Image%202021-12-12%20at%208.48.32%20PM.jpeg)
## - Demo 
Demo for ultrasonic and real time clock [Link](https://drive.google.com/file/d/1ngfxHc1YyZAuZVRz9Wq4pAjel6HeNgwg/view?usp=sharing)  

Demo for the water solenoid valve [Link](https://drive.google.com/file/d/1YRLaozv99t5Bi9kvaYcjjHZ7MpLMmAtr/view?resourcekey). 

Demo for the 5v DC [Link](https://drive.google.com/file/d/1hOTsf1hHnamgvyY9wUv3g-ecQ5YjsYzG/view?resourcekey)

Demo for the servo motor [Link](https://drive.google.com/file/d/1rENHwJuJgQNq3aCMhqf0htWsmoWDkImU/view?resourcekey)

Demo for weight sensor [Link](https://drive.google.com/file/d/17DasnVO54pVgVvfpmCNwtu4HX-maE9oo/view?resourcekey)


## - References
[HX711 and stm32](https://github.com/nimaltd/HX711)   

[electronic parts seller #1](https://ram-e-shop.com/).  

[electronic parts seller #2](https://store.fut-electronics.com/).   

[ESP32 Telegram](https://randomnerdtutorials.com/telegram-control-esp32-esp8266-nodemcu-outputs/).   

[ESP32 Uart communication](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html).   
 
[stm32 and servoMotors](https://controllerstech.com/servo-motor-with-stm32/).   

