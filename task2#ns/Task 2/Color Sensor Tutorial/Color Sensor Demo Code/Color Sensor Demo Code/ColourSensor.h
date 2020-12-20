#define S0  0
#define S1  1
#define S2  2
#define S3  3
#define OUT 0
#define BLUE 5
#define GREEN 6
#define RED  7

#include "ColourSensor.c"

uint16_t red, blue, green;
uint16_t pulse;//You can declare here global variable as per your requirement

void rgb_port_config (void);
void color_sensor_pin_config(void);
void color_sensor_pin_interrupt_init(void); //Interrupt 0 enable
//ISR for color sensor
ISR(INT0_vect); // Pass the timer number in place of n in INTn_vect
void init_devices(void);
//Filter Selection
void filter_red(void);    //Used to select red filter
void filter_green(void); //Used to select green filter
void filter_blue(void);  //Used to select blue filter
void filter_clear(void); //select no filter
//Color Sensing Scaling
void color_sensor_scaling();
void red_read(void);
void green_read(void);
void blue_read(void);
void pulseCount(void);