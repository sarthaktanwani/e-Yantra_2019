/*
 * GlobalVariables.cpp
 *
 * Created: 24-Feb-19 6:12:20 AM
 *  Author: PARTH
 */ 

#include <avr/io.h>

#include "GlobalVariables.h"

long map(long x, long in_min, long in_max, long out_min, long out_max)				//Similar to Arduino map function
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
