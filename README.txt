AUTHOR:  David OBERLEITNER, ic22b011@technikum-wien.at

DATE:    October 18, 2023

SUMMARY: This project aims to fullfil the Timer Task.

## Circuitry

You do not need to add/change any components or connection wires to the Nucleo Development Board. 

---

## Pins

The following pins are used for this project:

+ B3 is configured as output to drive the small green LED on the dev board
+ A6, A8 are configured as outputs to drive the RGB LED (red, green)

---

## Timers

The following timers are used for this project: 

+ TIM6 and TIM7

---

## Usage

The following steps outline the usage of this project:

+ Unzip the project
+ Flash the project onto the Nucleo Development Board
+ The small green LED is switched on for 4 seconds by non-blocking timer function 
+ Meanwhile the blocking function makes the big green LED blink
+ Small green LED is switched off independently by interrupt callback function 
+ In case of timer initialization error, red LED is switched on 

---

## Notes and Known Issues

+ Sleep mode for blocking function did not work and was replaced by blocking loop. 
+ Logic Analyzer screenshots added to show the timers' deviation. 
