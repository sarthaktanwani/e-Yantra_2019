/* 
* Team Id:#NS0039
* Author List: Madhur Raghav, Ayush Aggarwal, Prakhar Gupta, Sarthak Tanwani 
* Filename: main.cpp
* Theme: Nutty Squirrel
* Functions: minDistance(dist[],sptSet[]), dijkstra(graph[V][V], src, row), obstacleSearch(), obstacleFound(), printPath(int,int[],int), printSolution(int[], int,int[], int, int), 
*			robo_movement(int& , int& , int& ), orientation_check(), final_zone_1(), final_zone_2(), final_zone_3(), final_no_nut(), ServoSetup(), buzzer_init(), buzzerON(),
*			rgb_port_config (void), color_sensor_pin_config(void), color_sensor_pin_interrupt_init(void), init_devices(void), filter_red(void), filter_green(void), filter_blue(void),
*			filter_clear(void), color_sensor_scaling(), red_read(void), green_read(void), blue_read(void), pulseCount(void), isGreen(void), isBlue(void), isRed(void)
*			ISR(INT0_vect), ServoSetup(), elbow_write_angle(int), palm_write_angle(int), pick(), place(), , PID(), motor_control(), velocity(int, int), line_follow(), 
*			adc_read(uint8_t), read_sensor_value(), right_turn_wls(int), , nodedetect(void), forward(void), left(void), right(void), forward_wls(unsigned char ),
*			right_turn_wls(int), left_turn_wls(int), line_follow(), PWM_init(), velocity(left_motor_velocity, right_motor_velocity), read_sensor_value(), PID(), motor_control()
* Global Variables: red, blue, green, pulse, forwardtime, turntime, lsensor, msensor, rsensor, psensor, initial_motor_speed, Kp,Ki,Kd, error, P, I, D, PID_value, previous_error,*					previous_I, node, final_node, current_node, previous_node,distance_from_obstacle, object_in_front, first_red,first_green,first_blue, user, loop, 
*					left_motor_speed, right_motor_speed,
*
*/

#define  F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include "GlobalVariables.h"

int red;		//red_pulse_count stores the pulse of red colour after applying the red_filter()
int blue;		//green_pulse_count stores the pulse of red colour after applying the green_filter()
int green;		//blue_pulse_count stores the pulse of red colour after applying the blue_filter()
int pulse;		//You can declare here global variable as per your requirement
int forwardtime = 350, turntime = 300;
int lsensor, msensor, rsensor, psensor;	// variables for storing readings from leftmost IR sensor, middle IR sensor and rightmost IR sensor
int initial_motor_speed = 200;			//It sets minimum speed of the bot is there is zero error
float Kp=50,Ki=20,Kd=2;
float error=0, P=0, I=0, D=0, PID_value=0;
float previous_error=0, previous_I=0;
int node  = 0;
int final_node;			// variable stores the number of final node for traversing across the arena
int current_node=1;		// variable stores the number of current node for traversing across the arena
int previous_node=0;	// variable stores the number of previous node for traversing across the arena

int distance_from_obstacle;		//It stores the distance from object in front. It less than equal to 100 and garbage value if greater than 100
char object_in_front[10];		//It stores information about obstacle. It is equal to "obstacle" if it finds one otherwise "nothing"

int first_red=0,first_green=0,first_blue=0;
int user = 16;
int loop = 0;
int left_motor_speed;
int right_motor_speed;

int minDistance(int dist[],bool sptSet[]);
void dijkstra(bool graph[V][V], int src, int row);
char* obstacleSearch();
void obstacleFound();
void printPath(int,int[],int);
int printSolution(int[], int,int[], int, int);
void robo_movement(int& , int& , int& );
void orientation_check();
void final_zone_1();
void final_zone_2();
void final_zone_3();
void final_no_nut();
void ServoSetup();
void buzzer_init();
void buzzerON();

void rgb_port_config (void);
void color_sensor_pin_config(void);
void color_sensor_pin_interrupt_init(void);
void init_devices(void);
void filter_red(void);
void filter_green(void);
void filter_blue(void);
void filter_clear(void);
void color_sensor_scaling();
void red_read(void);
void green_read(void);
void blue_read(void);
void pulseCount(void);
int isGreen(void);
int isBlue(void);
int isRed(void);
ISR(INT0_vect);
void ServoSetup();
void elbow_write_angle(int);
void palm_write_angle(int);
void pick();
void place();

void PID();
void motor_control();
void velocity(int, int);
void line_follow();
uint16_t adc_read(uint8_t);
void read_sensor_value();
void right_turn_wls(int);

bool nodedetect(void);
void forward(void);
void left(void);
void right(void);
void forward_wls(unsigned char );
void right_turn_wls(int);
void left_turn_wls(int);
void line_follow();
void PWM_init();
void velocity(int left_motor_velocity, int right_motor_velocity);
void adc_init();
uint16_t adc_read(uint8_t ch);
void read_sensor_value();
void PID();
void motor_control();

/*
ADC Channel 4,5 and 6 for White LIne Sensor
ADC Channel 3 for Sharp Sensor
*/

bool graph[V][V] =	  { {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //Adjacency Matrix for given path
						{1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0}, //1 where there is a direct path between Nodes
						{0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0}, //0 where there is no direct connection
						{0,0,1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
						{0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
						{0,0,0,1,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
						{0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
						{0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
						{0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
						{0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
						{0,0,0,0,0,0,0,0,0,1,0,1,0,0,1,0,0,0,0,0,0,0,0},
						{0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
						{0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0},
						{0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0},
						{0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,1},
						{0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0},
						{0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0},
						{0,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,0,0,0,0},
						{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0},
						{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0},
						{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0},
						{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1},
						{0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0} };

/*
*
* Function Name: PWM_init
* Input: none
* Output: void
* Logic: Initializes PWM pins for motor (wheels)
* Example Call: PWM_init()
*
*/
void PWM_init()
{

  TCCR1A |= ((1<<WGM11)|(1<<WGM10)|(1<<COM1B1)|(1<<COM1A1));
  TCCR1B |= ((1<<CS10)|(1<<CS11));
  TCCR3A |= ((1<<WGM31)|(1<<WGM30)|(1<<COM3B1)|(1<<COM3A1));
  TCCR3B |= ((1<<CS30)|(1<<CS31));
  DDRB |= ((1<<LeftMotorPin1)|(1<<LeftMotorPin2)); 
  DDRE |= (1<<RightMotorPin1)|(1<<RightMotorPin2);
  
}
/*
*
* Function Name: velocity
* Input: int left_motor_velocity, int right_motor_velocity
* Output: void
* Logic: writes motor speed to motors (wheels)
* Example Call: velocity(150, 150)
*
*/
void velocity(int left_motor_velocity, int right_motor_velocity)
{
  if (left_motor_velocity > 0)
  {
    LeftMotorPin1REG = (left_motor_velocity > PWM_MAX) ? PWM_MAX: left_motor_velocity;
    LeftMotorPin2REG = 0;
  }
  else if (left_motor_velocity < 0)
  {
    left_motor_velocity *= (-1);
    LeftMotorPin1REG = 0;
    LeftMotorPin2REG = (left_motor_velocity > PWM_MAX) ? PWM_MAX : left_motor_velocity;
  }
  if (right_motor_velocity > 0)
  {
    RightMotorPin1REG = (right_motor_velocity > PWM_MAX) ? PWM_MAX : right_motor_velocity;
    RightMotorPin2REG = 0;
  }
  else if (right_motor_velocity < 0)
  {
    right_motor_velocity *= (-1);
    RightMotorPin1REG = 0;
    RightMotorPin2REG = (right_motor_velocity > PWM_MAX) ? PWM_MAX : right_motor_velocity;
  }
}

int main(void)
{
	rgb_port_config();
	color_sensor_pin_config();
	init_devices();
	color_sensor_scaling();
	PWM_init();
	adc_init();
	buzzer_init();
	user=16;
	dijkstra(graph,0,user);        //Dijkstra function will  take bot from 0th node to 16th node
	for( loop=0;loop<=5;loop++)
	{
		if((isGreen() == 1) && (isBlue() == 1) && (isRed() == 1))		     // If not will not be present then this function will be called
		{
			if(isGreen())
			{
				pick();
				final_zone_2();
			}
			else if(isBlue())
			{
				pick();
			}
			else if(isRed())
			{
				pick();
				if(first_red<2)
				final_zone_1();
				else
				final_zone_3();
			}
		}
		else
		{
			final_no_nut();
		}
		
	}
	buzzerON();  // after complete traversal of the bot, the buzzer will rung up.
	return 0;
}

/*
*
* Function Name: ServoSetup
* Input: none
* Output: void
* Logic: initializes pins for servo motor
* Example Call: ServoSetup()
*
*/
void ServoSetup()
{
	DDRH |= (1<<elbowPin);	/* Make OC4A pin as output */				//elbow sevo motor pin is PH3 or Pin 6 of the Arduino Mega
	TCNT4 = 0;		/* Set timer1 count zero */
	ICR4 = 2499;		/* Set TOP count for timer1 in ICR1 register */
	/* Set Fast PWM, TOP in ICR4, Clear OC4A on compare match, clk/64 */
	TCCR4A = (1<<WGM41)|(1<<COM4A1);
	TCCR4B = (1<<WGM42)|(1<<WGM43)|(1<<CS40)|(1<<CS41);
	
	DDRL |= (1<<palmPin);	/* Make OC1A pin as output */				//palm servo motor pin is PL3 or Pin 46 of the Arduino Mega
	TCNT5 = 0;		/* Set timer1 count zero */
	ICR5 = 2499;		/* Set TOP count for timer1 in ICR1 register */
	/* Set Fast PWM, TOP in ICR4, Clear OC5A on compare match, clk/64 */
	TCCR5A = (1<<WGM51)|(1<<COM5A1);
	TCCR5B = (1<<WGM52)|(1<<WGM53)|(1<<CS50)|(1<<CS51);
}
/*
*
* Function Name: elbow_write_angle
* Input: int angle
* Output: void
* Logic: takes in a value in angle (in degrees) and rotates the elbow servo motor to that angle
* Example Call: elbow_write_angle(90)
*
*/
void elbow_write_angle(int angle)
{
	OCR4A = map(angle, 0, 180, 65, 300);		//65 refers to 0 degree angle in servo and 300 refers to 180 degree angle in elbow servo motor
}

/*
*
* Function Name: palm_write_angle
* Input: int angle
* Output: void
* Logic: takes in a value in angle (in degrees) and rotates the palm servo motor to that angle
* Example Call: palm_write_angle(90)
*
*/
void palm_write_angle(int angle)
{
	OCR5A = map(angle, 0, 180, 65, 300);		//65 refers to 0 degree angle in servo and 300 refers to 180 degree angle in elbow servo motor
}

/*
*
* Function Name: adc_init
* Input: none
* Output: void
* Logic: initialize the ADC peripheral in AtMega2560
* Example Call: adc_init()
*
*/
void adc_init()
{
	// AREF = AVcc
	ADMUX = (1<<REFS0);
	
	// ADC Enable and prescaler of 128
	// 16000000/128 = 125000
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

/*
*
* Function Name: adc_read()
* Input: uint8_t ch
* Output: uint16_t
* Logic: takes in channel number and outputs the Analog value at the input of that channel
* Example Call: uint16_t value = adc_read(0) ; Assigns the analog value at channel 0 to variable 'value'
*
*/
uint16_t adc_read(uint8_t ch)
{
	// select the corresponding channel 0~7
	// ANDing with ’7? will always keep the value
	// of ‘ch’ between 0 and 7
	ch &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
	
	// start single convertion
	// write ’1? to ADSC
	ADCSRA |= (1<<ADSC);
	
	// wait for conversion to complete
	// ADSC becomes ’0? again
	// till then, run loop continuously
	while(ADCSRA & (1<<ADSC));
	
	return (ADC);
}

/*
*
* Function Name: pick
* Input: void
* Output: void
* Logic: controls the elbow servo motor and the palm servo motor together to pick up the nut
* Example Call: pick()
*
*/
void pick()
{
	_delay_ms(250);
	elbow_write_angle(elbowDown);
	_delay_ms(250);
	palm_write_angle(closePalm);
	_delay_ms(250);
	elbow_write_angle(elbowUp);
	_delay_ms(250);
}

/*
*
* Function Name: place
* Input: void
* Output: void
* Logic: controls the elbow servo motor and the palm servo motor together to place up the nut
* Example Call: place()
*
*/
void place()
{
	_delay_ms(250);
	elbow_write_angle(elbowDown);
	_delay_ms(250);
	palm_write_angle(openPalm);
	_delay_ms(250);
	elbow_write_angle(elbowUp);
	_delay_ms(250);
}

/*
*Function Name: nodedetect
*Input: void
*Output: bool
*Logic: Returns true if node is detected or otherwise returns false
*Example call: nodedetect()
*/
bool nodedetect(void)
{
	int n = 0;
	if (psensor <= 99)
	{
		return true;
	}
	if (n == 5 && ((lsensor == 255 && msensor == 255 && rsensor == 255)||(psensor<=99)))
	{
		return true;
	}
	
	else if ((lsensor == 255 && msensor == 255 && rsensor == 255)||(psensor<=99))
	{
		n++;
	}
	else
	{
		return false;
	}
	return false;
}


/*
*
* Function Name: forward
* Input: void
* Output: void
* Logic: moves the bot in the forward direction by setting both wheels (motors) to high positive value
* Example Call: forward()
*
*/
void forward(void)
{
	velocity(PWM_MAX, PWM_MAX);
}

/*
*
* Function Name: left
* Input: void
* Output: void
* Logic: moves the bot in the left direction by setting left wheel to negative high value and right wheel to positive high value
* Example Call: left()
*
*/
void left(void)
{
	int linear_velocity_left, linear_velocity_right;
	linear_velocity_left = -PWM_MAX;
	linear_velocity_right = PWM_MAX;
	velocity(linear_velocity_left, linear_velocity_right);
}

/*
*
* Function Name: right
* Input: void
* Output: void
* Logic: moves the bot in the right direction by setting left wheel to positve high value and right wheel to negative high value
* Example Call: right()
*
*/
void right(void)
{
	int linear_velocity_left, linear_velocity_right;
	linear_velocity_left = PWM_MAX;
	linear_velocity_right = -PWM_MAX;
	velocity(linear_velocity_left, linear_velocity_right);
}

/*
*
* Function Name: forward_wls
* Input: node
* Output: void
* Logic: Uses white line sensors to go forward by the number of nodes specified
* Example Call: forward_wls(2); //Goes forward by two nodes
*
*/
void forward_wls(unsigned char node = 1)
{
	do
	{
		line_follow();
	} while (--node);

}

/*
*
* Function Name: right_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn right until black line is encountered
* Example Call: right_turn_wls(); //Turns right until black line is encountered
*/
void right_turn_wls(int flag = false)
{
	if (flag == true)
	{
		forward();
		do
		{
			read_sensor_value();
		} while (msensor);
	}
	if (flag == false)
	{
		right();
		_delay_ms(100);
	}
	do
	{
		right();
		read_sensor_value();
	} while (msensor != PWM_MAX);
}

/*
*
* Function Name: left_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn left until black line is encountered
* Example Call: left_turn_wls(); //Turns right until black line is encountered
*
*/
void left_turn_wls(int flag = false)
{
	if (flag == true)
	{
		forward();
		do
		{
			read_sensor_value();
		} while (msensor);
	}
	if (flag == false)
	{
		left();
		_delay_ms(100);
	}
	do
	{
		left();
		read_sensor_value();
	} while (msensor != PWM_MAX);
}

/*
*Function Name:line_follow
*Input: void
*Output: void
*Logic: Implements Line-following algorithm
*Example call: line_follow();
*/
void line_follow()
{
	forward();
	_delay_ms(50);
	do
	{
		read_sensor_value();
		PID();
		motor_control();
		_delay_ms(5);
		if(strcmp(obstacleSearch(), "obstacle") == 0)
		{
			break;
		}
	} while (!(nodedetect()));
	if (strcmp(obstacleSearch(), "obstacle") == 0)
	{
		pulseCount();
		obstacleFound();
	}
	else
	{
		forward();
		_delay_ms(155);
	}
}
	
/*
*Function Name: minDistance
*Input: int, bool
*Output: int
*Logic: This function finds the vertex with the minimum distance value from the set of vertices not yet included in shortest path tree
*Example Call: int a =minDastance(int dist[],bool sptSet[] )
*/
int minDistance(int dist[],bool sptSet[])
{

	// Initialize min value
	int min = INT_MAX, min_index = 0;
	for (int v = 0; v < V; v++)
	{
		if (sptSet[v] == false && dist[v] <= min)
		{
			min = dist[v], min_index = v;
		}
	}
	return min_index;
}

/*
*Function Name: robo_movement
*Input: int, int, int
*Output; void
*Logic: This function is controlling the orientation of the bot while traversing the arena
*Example Call: robo_movement(1,19,20)
*/
void robo_movement(int &final_node, int &current_node, int &previous_node)
{
	if (final_node == 1)
	{
		if (current_node == 0)
		{
			if (previous_node == 0)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}

		}
		else if (current_node == 4)
		{
			if (previous_node == 5)
			{
				line_follow();
				forward();                                                             //CHECK THIS
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 15)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 2)
		{
			if (previous_node == 3)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 13)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 19)
		{
			if (previous_node == 18)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 20)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 2)
	{
		if (current_node == 3)
		{
			if (previous_node == 5)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 6)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 13)
		{
			if (previous_node == 17)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 12)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 1)
		{
			if (previous_node == 4)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 19)
			{
				forward_wls();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 0)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}

	}
	else if (final_node == 3)
	{
		if (current_node == 2)
		{
			if (previous_node == 1)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 13)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 5)
		{
			if (previous_node == 4)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 9)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 6)
		{
			if (previous_node == 7)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 4)
	{
		if (current_node == 15)
		{
			if (previous_node == 21)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 14)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 1)
		{
			if (previous_node == 19)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 2)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 0)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 5)
		{
			if (previous_node == 9)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 3)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 5)
	{
		if (current_node == 4)
		{
			if (previous_node == 1)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 15)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 3)
		{
			if (previous_node == 2)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 6)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 9)
		{
			if (previous_node == 10)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 6)
	{
		if (current_node == 7)
		{
			if (previous_node == 12)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 8)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 3)
		{
			if (previous_node == 2)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 5)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 7)
	{
		if (current_node == 12)
		{
			if (previous_node == 13)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 16)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 6)
		{
			if (previous_node == 3)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 8)
		{
			if (previous_node == 7)
			{
				line_follow();                                                     //THIS
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 8)
	{
		if (current_node == 7)
		{
			if (previous_node == 6)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 12)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 9)
	{
		if (current_node == 10)
		{
			if (previous_node == 11)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 14)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 5)
		{
			if (previous_node == 4)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 3)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 10)
	{
		if (current_node == 14)
		{
			if (previous_node == 15)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 22)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 9)
		{
			if (previous_node == 5)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 11)
		{
			if (previous_node == 10)
			{
				line_follow();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 11)
	{
		if (current_node == 10)
		{
			if (previous_node == 9)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 14)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 12)
	{
		if (current_node == 7)
		{
			if (previous_node == 8)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 6)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 16)
		{
			if (previous_node == 17)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 13)
		{
			if (previous_node == 17)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 2)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 13)
	{
		if (current_node == 2)
		{
			if (previous_node == 1)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 3)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 17)
		{
			if (previous_node == 16)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 18)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 12)
		{
			if (previous_node == 7)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 16)
			{
				forward_wls();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 14)
	{
		if (current_node == 10)
		{
			if (previous_node == 11)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 9)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 22)
		{
			if (previous_node == 21)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 15)
		{
			if (previous_node == 4)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 21)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 15)
	{
		if (current_node == 4)
		{
			if (previous_node == 5)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 1)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 21)
		{
			if (previous_node == 20)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 22)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 14)
		{
			if (previous_node == 22)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 10)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 16)
	{
		if (current_node == 12)
		{
			if (previous_node == 13)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 7)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 17)
		{
			if (previous_node == 18)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 13)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 17)
	{
		if(current_node == 13)
		{
			if (previous_node == 12)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 2)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 18)
		{
			if (previous_node == 19)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 16)
		{
			if (previous_node == 12)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 18)
	{
		if (current_node == 17)
		{
			if (previous_node == 13)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 16)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 19)
		{
			if (previous_node == 1)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 20)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 19)
	{
		if (current_node == 1)
		{
			if (previous_node == 4)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 0)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 4)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		if (current_node == 18)
		{
			if (previous_node == 17)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 20)
		{
			if (previous_node == 21)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 20)
	{
		if (current_node == 21)
		{
			if (previous_node == 15)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 22)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 19)
		{
			if (previous_node == 1)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 18)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if (final_node == 21)
	{
		if (current_node == 15)
		{
			if (previous_node == 14)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 4)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 22)
		{
			if (previous_node == 14)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 20)
		{
			if (previous_node == 19)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
	else if(final_node == 22)
	{
		if (current_node == 14)
		{
			if (previous_node == 15)
			{
				line_follow();
				left_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 10)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
		}
		else if (current_node == 21)
		{
			if (previous_node == 20)
			{
				line_follow();
				forward();
				previous_node = current_node;
				current_node = final_node;
			}
			else if (previous_node == 15)
			{
				line_follow();
				right_turn_wls();
				previous_node = current_node;
				current_node = final_node;
			}
		}
	}
}

/*
*
* Function name : orientation_check
* Input :void
* Output :void
* Logic : The orientation of the bot will be controlled using numbering of the nodes.
* Example Call : orientaton_check(); // turn the face position of the robot towards the nut
*/
void orientation_check()
{
	if((previous_node-current_node-1)==0)
	{
		right_turn_wls();
	}
	else if((current_node-previous_node-1)==0)
	{
		left_turn_wls();
	}
}

/*
*
* Function name : final_no_nut
* Input :void
* Output :void
* Logic : This function will be called when the bot encounter an absence of nut in the pickup zone
* Example Call : final_no_nut(); // will traverse the bot from one node of pickup zone to the next node of pickup zone
*/
void final_no_nut()
{
	right_turn_wls();
	line_follow();
	left_turn_wls();
	user=user+1;
	if(isGreen())
	{
		pick();
		final_zone_2();
	}
	else if(isBlue())
	{
		pick();
		final_zone_1();
	}
	else if(isRed())
	{
		pick();
		final_zone_3();
	}
	else if((isGreen() == 0) && (isBlue() == 0) && (isRed() == 0))
	{
		final_no_nut();
	}
}

/*
*
* Function name : final_zone_1
* Input :void
* Output :void
* Logic : This function will help in placing of nut of one of the three colours according to the given deposit zone
* Example Call : final_zone_1(); // will traverse the bot to the deposit zone and back to the next picking node
*/
void final_zone_1()
{
	if(first_red==0)   //place at 9th node
	{
		dijkstra(graph,user,9);
		user=9;
		if(previous_node==5)
		{
			right_turn_wls();
			place();
		}
		else
		{
			left_turn_wls();
			place();
		}
		right_turn_wls();
		previous_node=9;
		current_node=5;
		user=5;
	}
	
	else if(first_red==1)//place at 11th node
	{
		dijkstra(graph,user,11);
		user=11;
		place();
		right_turn_wls();
		previous_node=11;
		current_node=10;
		user=10;
	}
	dijkstra(graph,user,loop+16);
	user=loop+16;
	orientation_check();
	first_red++;
}

/*
*
* Function name : final_zone_2
* Input :void
* Output :void
* Logic : This function will help in placing of nut of of of the three colours according to the given deposit zone
* Example Call : final_zone_2(); // will traverse the bot to the deposit zone and back to the next picking node
*/
void final_zone_2()
{
	if(first_green==0)//place at 26th node
	{
		dijkstra(graph,user,26);
		user=26;
		place();
		right_turn_wls();
		previous_node=26;
		current_node=25;
		user=25;
	}
	
	else if(first_green==1)//place at 27th node
	{
		dijkstra(graph,user,8);
		user=8;
		place();
		right_turn_wls();
		previous_node=26;
		current_node=25;
		user=25;
	}
	dijkstra(graph,user,loop+16);
	user=loop+16;
	orientation_check();
	first_green++;
}

/*
*
* Function name : final_zone_3
* Input :void
* Output :void
* Logic : This function will help in placing of nut of one of the three colours according to the given deposit zone
* Example Call : final_zone_3(); // will traverse the bot to the deposit zone and back to the next picking node
*/
void final_zone_3()
{
	if(first_blue==0)//place at 6th node
	{
		dijkstra(graph,user,6);
		user=6;
		if(previous_node==7)
		{
			right_turn_wls();
			place();
		}
		else
		{
			left_turn_wls();
			place();
		}
		right_turn_wls();
		previous_node=6;
		current_node=7;
		user=7;
	}
	
	else if(first_blue==1)//place at 8th node
	{
		dijkstra(graph,user,8);
		user=8;
		place();
		right_turn_wls();
		previous_node=8;
		current_node=7;
		user=7;
	}
	dijkstra(graph,user,loop+16);
	user=loop+16;
	orientation_check();
	first_blue++;
}


/*
*Function Name: printPath()
*Input: int, int, int
*Output: void
*Logic: It is a recursive function that prints out the shortest path to the destination from the source using parent array
*Example Call: printPath(int src, int parent[], int destination)
*/
void printPath(int src,int parent[], int j)
{
	if (parent[j] == -1)
	{
		return;
	}
	printPath(src,parent, parent[j]);
	final_node = j;
	robo_movement(final_node, current_node, previous_node);
}

/*
*Function Name: printSolution
*Input: int, int, int, int, int
*Output: int
*Logic: Prints the shortest path calculated in the dijkstra() function on the console
*Example Call: printSolution(int dist[], int n,int parent[], int src, int row)
*/
int printSolution(int dist[], int n,int parent[], int src, int row)
{
	printPath(src,parent, row);
	return 0;
}

/*
*Function Name: dijkstra
*Input: bool[][], int, int
*Output: void
*Logic: Funtion that implements Dijkstra's single source shortest path algorithm for a graph represented using adjacency matrix representation
*Example Call: dijkstra(bool graph[V][V], int src, int row)
*/
void dijkstra(bool graph[V][V], int src, int row)
{
	// The output array. dist[i]
	// will hold the shortest
	// distance from src to i
	int dist[V];
	// sptSet[i] will true if vertex
	// i is included / in shortest
	// path tree or shortest distance
	// from src to i is finalized
	bool sptSet[V];
	// Parent array to store
	// shortest path tree
	int parent[V];
	// Initialize all distances as
	// INFINITE and stpSet[] as false
	for (int i = 0; i < V; i++)
	{
		parent[src] = -1;
		dist[i] = INT_MAX;
		sptSet[i] = false;
	}
	// Distance of source vertex
	// from itself is always 0
	dist[src] = 0;
	// Find shortest path
	// for all vertices
	for (int count = 0; count < V - 1; count++)
	{
		// Pick the minimum distance
		// vertex from the set of
		// vertices not yet processed.
		// u is always equal to src
		// in first iteration.
		int u = minDistance(dist, sptSet);
		// Mark the picked vertex
		// as processed
		sptSet[u] = true;
		// Update dist value of the
		// adjacent vertices of the
		// picked vertex.
		for (int v = 0; v < V; v++)
		// src to v through u is smaller
		// than current value of
		// dist[v]
		if (!sptSet[v] && graph[u][v] &&
		dist[u] + graph[u][v] < dist[v])
		{
			parent[v] = u;
			dist[v] = dist[u] + graph[u][v];
		}
	}
}

/*
*Function Name: obstacleSearch
Input: void
Output: string
Logic: If sees obstacle within sensor range, returns "obstacle" otherwise returns "nothing"
Example Call: obstacleSearch()
*/
char* obstacleSearch()
{
	distance_from_obstacle = adc_read(3);
	if (distance_from_obstacle <= 100)
	{
		if (red > green && green >= blue )
		{
			strcpy(object_in_front, "obstacle");
		}
		return object_in_front;
	}
	else
	{
		strcpy(object_in_front, "nothing");
		return object_in_front;
	}
}

/*
*Function Name: obstacleFound
*Input: void
*Output: void
*Logic: It is called when a obstacle is found. It updates the adjacency matrix, previous_node, current_node and 
		final_node if a obstacle is found
*Example call: obstacleFound()
*/
void obstacleFound() 
{
	graph[current_node][final_node] = graph[final_node][current_node] = 0;
	right_turn_wls(false);// right turn to go back to the previous node
	line_follow();
	int temp;
	temp = previous_node;
	previous_node = final_node;
	final_node = temp;
	dijkstra(graph, current_node, final_node);
}

/*
*
*Function Name: read_sensorvalue
*Input: void
*Output void
*Logic: Reads the values of all the sensors and assigns to global variables
*Example call: read_sensorvalue();
*
*/
void read_sensor_value()
{
	lsensor=adc_read(4);
	msensor=adc_read(5);
	rsensor=adc_read(6);
	psensor=adc_read(7);
}

/*
*
*Function name: PID
*Input:void
*Output: void
*Logic: Calculates the PID value for the given sensor readings
*Example Call: PID();
*
*/
void PID(void)
{
	if (lsensor + rsensor + msensor)
	error = (-3 * lsensor + 3 * rsensor) / (lsensor + rsensor + msensor);
	else
	{
		error = previous_error;
	}
	P = error;
	I = I + previous_I;
	D = error - previous_error;

	PID_value = (Kp*P) + (Ki*I) + (Kd*D);
	previous_I = I;
	previous_error = error;

}

/*
*
*Function name: motor_control
*Input: void
*Output: void
*Logic: Drivers the motor at different speeds after calculating the speed
*Example call: motor_control()
*
*/
void motor_control()
{
    // Calculating the effective motor speed:
    left_motor_speed = initial_motor_speed-PID_value;
    right_motor_speed = initial_motor_speed+PID_value;
    velocity(left_motor_speed, right_motor_speed);
}

/*
*
* Function Name: rgb_port_config
* Input: void
* Output: void
* Logic: Initialize/configure the Red, Green and Blue LED port
* Example Call: rgb_port_config()
*
*/
void rgb_port_config (void)
{
  /*****************************************
    Define DDR and PORT values for the port on which RGB LED is connected
  ******************************************/
  DDRA |= 0xE0;
  PORTA |= 0xE0;
}

/*
*
* Function Name: color_sensor_pin_config
* Input: void
* Output: void
* Logic: Initialize/configure the colour sensor pins
* Example Call: color_sensor_pin_config()
*
*/
void color_sensor_pin_config(void)
{
  /*****************************************
    Define DDR and PORT values for the port on which Color sensor is connected
  ******************************************/
  DDRA |= (1 << S0) | (1 << S1) | (1 << S2) | (1 << S3);
  DDRD &= ~(1 << OUT);
  //PORTA |= 0x0F;
}

/*
*
* Function Name: color_sensor_pin_interrupt_init
* Input: void
* Output: void
* Logic: Initialize the colour sensor pin as interrupt which is triggered when pulses are received
* Example Call: color_sensor_pin_interrupt_init()
*
*/
void color_sensor_pin_interrupt_init(void) //Interrupt 0 enable
{
  cli(); //Clears the global interrupt
  EICRA |= (1 << ISC01); // INT0 is set to trigger with falling edge
  EICRA &= ~(1 << ISC00);
  EIMSK |= (1 << INT0); // Enable Interrupt INT0 for color sensor
  sei(); // Enables the global interrupt
}


/*
*
* Function Name: ISR for color sensor
* Logic: Increments the pulses whenever they are received
*
*/
ISR(INT0_vect) // Pass the timer number in place of n in INTn_vect
{
  //increment on receiving pulse from the color sensor
  pulse++;
}

void init_devices(void)
{
  cli(); //Clears the global interrupt
  //Initialize all the ports here
  color_sensor_pin_interrupt_init();
  sei();   // Enables the global interrupt
}

//Filter Selection
void filter_red(void)    //Used to select red filter
{
  //Filter Select - red filter
  PORTA &= ~(1 << S2); //set S2 low
  PORTA &= ~(1 << S3); //set S3 low
}

void filter_green(void) //Used to select green filter
{
  //Filter Select - green filter
  PORTA |= (1 << S2); //set S2 High
  PORTA |= (1 << S3); //set S3 High
}

void filter_blue(void)  //Used to select blue filter
{
  //Filter Select - blue filter
  PORTA &= ~(1 << S2); //set S2 low
  PORTA |= (1 << S3); //set S3 High
}

void filter_clear(void) //select no filter
{
  //Filter Select - no filter
  PORTA |= (1 << S2); //set S2 High
  PORTA &= ~(1 << S3); //set S3 Low
}

//Color Sensing Scaling
void color_sensor_scaling()   //This function is used to select the scaled down version of the original frequency of the output generated by the color sensor, generally 20% scaling is preferable, though you can change the values as per your application by referring datasheet
{
  //Output Scaling 20% from datasheet

  PORTA |= (1 << S0); //set S0 high
  PORTA |= (1 << S1); //set S1 high
}

void red_read(void) // function to select red filter and display the count generated by the sensor on LCD. The count will be more if the color is red. The count will be very less if its blue or green.
{
  //Red
  filter_red();//select red filter
  pulse = 0;//reset the count to 0
  _delay_ms(100);//capture the pulses for 100 ms or 0.1 second
  red = pulse;//store the count in variable called red
}

void green_read(void) // function to select green filter and display the count generated by the sensor on LCD. The count will be more if the color is green. The count will be very less if its blue or red.
{
  //Green
  filter_green();//select green filter
  pulse = 0;//reset the count to 0
  _delay_ms(100);//capture the pulses for 100 ms or 0.1 second
  green = pulse;//store the count in variable called green
}

void blue_read(void) // function to select blue filter and display the count generated by the sensor on LCD. The count will be more if the color is blue. The count will be very less if its red or green.
{
  //Blue
  filter_blue(); //select blue filter
  pulse = 0; //reset the count to 0
  _delay_ms(100); //capture the pulses for 100 ms or 0.1 second
  blue = pulse;  //store the count in variable called blue
}

/*
*
*Function Name: pulseCount
*Input: void
*Output void
*Logic: Reads the values of the proximity and colour sensors and assigns to global variables
*Example call: pulseCount();
*
*/
void pulseCount(void)
{
	red_read();
	blue_read();
	green_read();
}

/*
*Function Name: isGreen
*Input: void
*Output: bool
*Logic: Check the colour of object in front and return true if it is true and false if it is not green
*Example Call: isGreen()
*/
int isGreen()
{
	distance_from_obstacle = adc_read(3);
	if (distance_from_obstacle <= 60)
	{
		if (green > red && green > blue )
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	return -1;
}
/*
*Function Name: isRed
*Input: void
*Output: bool
*Logic: Check the colour of object in front and return true if it is red and false if it is not red
*Example Call: isRed()
*/
int isRed()
{
	distance_from_obstacle = adc_read(3);
	if (distance_from_obstacle <= 60)
	{
		if (red > green && red > blue )
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}
}

/*
*Function Name: isBlue
*Input: void
*Output: bool
*Logic: Check the colour of object in front and return true if it is red and false if it is not red
*Example Call: isBlue()
*/
int isBlue()
{
	distance_from_obstacle = adc_read(3);
	if (distance_from_obstacle <= 60)
	{
		if (blue > green && blue > red )
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}
}

/*
*
* Function Name: buzzer_init()
* Input: void
* Output: void
* Logic: Initializes the buzzer
* Example Call: buzzer_init()
*
*/
void buzzer_init()
{
	DDRC |= (1<<BuzzerPin);			//Buzzer Pin on Port C pin 1 or pin 54 of the arduino Mega
	PORTC &= ~(1<<BuzzerPin);
}

/*
*
* Function Name: buzzerON()
* Input: void
* Output: void
* Logic: Switches on the buzzer
* Example Call: buzzerON()
*
*/
void buzzerON()
{
	PORTC |= (1<<BuzzerPin);
}