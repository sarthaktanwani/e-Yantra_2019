/*
**
  Team Id: 0039
  Author List: Madhur Raghav, Sarthak Tanwani, Prakhar Gupta, Ayush Aggarwal
  Filename: main.c
  Theme: Nutty Squirrel
  Functions: PWM_init(), writeMotorSpeed(int), stop(), main()
  Global Variables: IN1, IN2, EN, IR, LimitSwitchUp, LimitSwitchDown, PWMREG, motorSpeed, goUp
**
*/

#define F_CPU  16000000UL
#include <avr/io.h>
#include <util/delay.h>

#define IN1 (1<<0)                  //Pin PB0 OR digital pin 8 of the arduino nano (and arduino also)
#define IN2 (1<<7)                  //Pin PD7 OR digital pin 7 of the arduino nano (and arduino uno also)
#define EN (1<<6)                   //Pin PD6 OR digital pin 6 of the arduino nano (and arduino uno also)
#define IR  (1<<4)                  //input for the IR sensor value; Pin PD4 OR digital pin 4 of the arduino nano
#define LimitSwitchUp (1<<2)        //PD2 of atmega328p OR Pin 2 of arduino uno/nano
#define LimitSwitchDown (1<<3)      //PD3 of atmega328p OR Pin 3 of arduino uno/nano
#define PWMREG  OCR0A

//these values were calibrated during experimentation
int motorSpeed = 100;
unsigned short int goUp = 1;		//Flag that stores thee status of whether the lift is in up state or down state

/*
**
  Function Name: PWM_init()
  Input: None
  Output: None
  Logic: This function is used to initialize the PWM registers to interface the motor driver IC
  Example Call: PWM_init()
**
*/
void PWM_init()
{
  TCCR0A |= ((1 << WGM01) | (1 << WGM00) | (1 << COM0B1) | (1 << COM0A1));
  TCCR0B |= ((1 << CS00) | (1 << CS01));
  DDRD |= EN;						//set OCR0A pin as output
}

/*
**
  Function Name: writeMotorSpeed(int)
  Input: speed (of type int)
  Output: None
  Logic: This function  is used to write analog value (of motorSpeed) to Motor Driver IC
  Example Call: writeMotorSpeed(motorSpeed)
**
*/
void writeMotorSpeed(int speed)		//Function to write motor speed on the motor driver IC
{
  PWMREG = motorSpeed;
}

/*
**
  Function Name: stop()
  Input: void 
  Output: void
  Logic: This function  is used to stop the lift motor
  Example Call: stop()
**
*/
void stop()
{
      PORTB &= ~IN1;				//Setting IN1 to LOW
      PORTD &= ~IN2;				//Setting IN2 to LOW
      writeMotorSpeed(0);
}

int main()
{
  PWM_init();
  DDRD &= ~IR;						//Setting PD4 as input
  DDRB |= IN1;						//Setting PB0 as output
  DDRD |= IN2;						//Setting PD7 as output
  DDRD |= LimitSwitchUp;
  DDRD |= LimitSwitchDown;
  while (1)
  {
    if (((PIND & IR) == 0) && goUp == 1) //Case when bot is on the lift and the lift needs to go up
    {
      _delay_ms(1000);					//Initial delay to make sure bot is completely on the lift
      PORTB |= IN1;						//Setting IN1 to HIGH
      PORTD &= ~IN2;					//Setting IN2 to LOW
      writeMotorSpeed(motorSpeed);
      while ((PIND & LimitSwitchUp) == 0);
      stop();
      goUp = 0;
    }
    else if (((PIND & IR) == 0) && goUp == 0) //Case when bot is on the lift and the lift needs to go down
    {
      _delay_ms(1000);					//Initial delay to make sure bot is completely on the lift
      PORTB &= ~IN1;					//Setting IN1 to LOW
      PORTD |= IN2;						//Setting IN2 to HIGH
      writeMotorSpeed(motorSpeed);
      while ((PIND & LimitSwitchDown) == 0);
      stop();
      goUp = 1;
    }
    else
    {
      stop();
    }
  }
}