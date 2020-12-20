/*
   Color_Sensor.c

   Created: 01-12-2018 14:45:11
    Author: e-Yantra Team

  This experiment demonstrates the use of Color Sensor.

  Micro-controller: atmega2560
  Optimization: -O0  (For more information read section: Selecting proper optimization
  options below figure 2.22 in the Software Manual)
*/

#define F_CPU 16000000UL //define F_CPU value
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "ColourSensor.h"
//#include "ColourSensor.c"



int main(void)
{
  rgb_port_config();
  color_sensor_pin_config();
  init_devices();
  color_sensor_scaling();
  while(1)
  {
    //Write your code here
	pulseCount();
      uint16_t max = (red > blue) ? ((red > green) ? red : green) : ((green < blue) ? blue : green);	//Cool way of finding the max value, I like this one!
      //uint16_t max = max(red, max(blue, green));														//Slightly cool way of finding max value from a 2 argument function
	  if(max < 1000)
	  {
		  PORTA |= (1<<RED)|(1<<GREEN)|(1<<BLUE);
	  }
      else if (max == red)
      {
        PORTA |= ((1 << BLUE) | (1 << GREEN));
        PORTA &= ~(1<<RED);
      }
      else if (max == green)
      {
        PORTA |= ((1 << BLUE) | (1 << RED));
        PORTA &= ~(1<<GREEN);
      }
      else if (max == blue)
      {
        PORTA |= ((1 << RED) | (1 << GREEN));
        PORTA &= ~(1<<BLUE);
      }
  }
}