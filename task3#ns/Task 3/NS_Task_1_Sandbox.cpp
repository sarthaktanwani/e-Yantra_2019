/*
*Team Id: 0039
* Author List: Madhur Raghav, Prakhar Gupta, Ayush Aggarwal, Sarthak Tanwani
* Filename: NS_Task_1_Sandbox.cpp
* Theme: Nutty Squirrel
* Functions: robo_movement,line_follow,minDistance,printPath,printSolution,dijkstra,pulseCount,read_sensorvalue,printsenesordata,
* 			 nodedetect,PID,motorcontrol,obstacleSearch,obstacleFound,forward_wls, left_turn_wls, right_turn_wls, Square, Task_1_1,Task_1_2
* Global Variables: lsensor, msensor, rsensor, initial_motor_speed, Kp,Ki,Kd, error, P, I, D , PID_value,
*					previous_error , previous_I,node, final_node, current_node, previous_node, red_pulse_count, green_pulse_count, blue_pulse_count
*					distance_from_obstacle, object_in_front, green, red;
*
*/
#include "NS_Task_1_Sandbox.h"
#include <stdio.h>
#include <limits.h>
#define V 23 
using namespace std;
int lsensor, msensor, rsensor;	// variables for storing readings from leftmost IR sensor, middle IR sensor and rightmost IR sensor
int initial_motor_speed = 200;	//It sets minimum speed of the bot is there is zero error 
float Kp = 60, Ki = 0, Kd = 0;	//Kp,Ki,Kd are the Proportional Constant, Integral constant and Derivative constant respectively
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int node = 0;
int final_node;			// variable stores the number of final node for traversing across the arena
int current_node=1;		// variable stores the number of current node for traversing across the arena
int previous_node=0;	// variable stores the number of previous node for traversing across the arena
int red_pulse_count;	//red_pulse_count stores the pulse of red colour after applying the red_filter()
int green_pulse_count;	//green_pulse_count stores the pulse of red colour after applying the green_filter()
int blue_pulse_count;	//blue_pulse_count stores the pulse of red colour after applying the blue_filter()
int distance_from_obstacle;	//It stores the distance from object in front. It less than equal to 100 and garbage value if greater than 100
string object_in_front;		//It stores information about obstacle. It is equal to "obstacle" if it finds one otherwise "nothing"
int green, red;		//They store the count of green and red nuts placed respectively
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
*Function Name: minDistance
*Input: int, bool
*Output: int
*Logic: This function finds the vertex with the minimum distance value from the set of vertices not yet included in shortest path tree
*Example Call: int a =minDastance(int dist[],bool sptSet[] )
*/
int minDistance(int dist[],bool sptSet[])
{
	// Initialize min value 
	int min = INT_MAX, min_index;
	for (int v = 0; v < V; v++)
		if (sptSet[v] == false && dist[v] <= min)
			min = dist[v], min_index = v;
	return min_index;
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
*Input: bool, int, int
*Output: void
*Logic: This Funtion implements Dijkstra's single source shortest path algorithm for a 
*		graph represented using adjacency matrix representation 
*Example Call: dijkstra(graph,current_node, final_node)
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
	printSolution(dist, V, parent, src,row);
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
	distance_from_obstacle = (int)ADC_Conversion(FRONT_IR_ADC_CHANNEL);
	filter_red();
	red_pulse_count = color_sensor_pulse_count;
	filter_green();
	green_pulse_count = color_sensor_pulse_count;
	filter_blue();
	blue_pulse_count = color_sensor_pulse_count;
	filter_clear();
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
void read_sensorvalue(void)
{
	lsensor = ADC_Conversion(1);
	msensor = ADC_Conversion(2);
	rsensor = ADC_Conversion(3);
	pulseCount();
}
/*
*
*Function name:printsensordata
*Input: void
*Output: void
*Logic: Prints the sensor data of IR Senosr on console;
*Example call:printsensordata();
*
*/
void printsensordata(void)
{
	cout << lsensor << " " << msensor << " " << rsensor << "\n";
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
	if (n == 5 && (lsensor == 255 && msensor == 255 && rsensor == 255))
	{
		return true;
	}
	
	else if ((lsensor == 255 && msensor == 255 && rsensor == 255))
	{
		n++;
	}
	else
	{
		return false;
	}
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
*Function name: motorcontrol
*Input: void
*Output: void
*Logic: Drivers the motor at different speeds after calculating the speed
*Example call: motorcontrol()
*
*/
void motorcontrol(void)
{
	int left_motor_speed = initial_motor_speed + PID_value;
	int right_motor_speed = initial_motor_speed - PID_value;
	velocity(left_motor_speed, right_motor_speed);
}
/*
*Function Name: obstacleSearch
Input: void
Output: string
Logic: If sees obstacle within sensor range, returns "obstacle" otherwise returns "nothing"
Example Call: obstacleSearch()
*/
string obstacleSearch()
{
	if (distance_from_obstacle <= 100)
	{
		if (red_pulse_count > green_pulse_count && green_pulse_count >= blue_pulse_count)
		{
			object_in_front = "obstacle";
		}
		return "obstacle";
	}
	else
	{
		object_in_front = "nothing";
		return "nothing";
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
	cout << current_node << " " << previous_node << " " << final_node;
	temp = previous_node;
	previous_node = final_node;
	final_node = temp;
	dijkstra(graph, current_node, final_node);
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
		pulseCount();
		read_sensorvalue();
		PID();
		motorcontrol();
		_delay_ms(5);
		if(obstacleSearch()=="obstacle")
		{
			break;
		}
	} while (!(nodedetect()));
	if (obstacleSearch() == "obstacle")
	{
		obstacleFound();
	}
	else
	{
		forward();
		_delay_ms(155);
	}
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
			read_sensorvalue();
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
		read_sensorvalue();
	} while (msensor != 255);
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
			read_sensorvalue();
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
		read_sensorvalue();
	} while (msensor != 255);
}
/*
*
* Function Name: Square
* Input: void
* Output: void
* Logic: Use this function to make the robot trace a square path on the arena
* Example Call: Square();
*/
void Square(void)
{
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
*Function Name: isGreen
*Input: void
*Output: bool
*Logic: Check the colour of object in front and return true if it is true and false if it is not green
*Example Call: isGreen()
*/
bool isGreen()
{
	if (distance_from_obstacle <= 60)
	{
		if (green_pulse_count > red_pulse_count && green_pulse_count > blue_pulse_count)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}
/*
*Function Name: isRed
*Input: void
*Output: bool
*Logic: Check the colour of object in front and return true if it is red and false if it is not red
*Example Call: isRed()
*/
bool isRed()
{
	if (distance_from_obstacle <= 60)
	{
		if (red_pulse_count > green_pulse_count && red_pulse_count > blue_pulse_count)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}
/*
*
* Function Name: Task_1_1
* Input: void
* Output: void
* Logic: Use this function to encapsulate your Task 1.1 logic
* Example Call: Task_1_1();
*
*/
void Task_1_1(void)
{

}
/*
*
* Function Name: Task_1_2
* Input: void
* Output: void
* Logic: Use this function to encapsulate your Task 1.2 logic
* Example Call: Task_1_2();
*/
void Task_1_2(void)
{
	forward_wls();
	dijkstra(graph, 1, 16);
	if (previous_node == 17)
	{
		forward_wls();
		forward();
		_delay_ms(50);
		right_turn_wls();
		stop();
		right();
	}
	else if (previous_node == 12)
	{
		forward_wls();
		forward();
		_delay_ms(100);
		stop();
	}
	_delay_ms(40);
	stop();
	if (isRed())
	{
		red++;
		_delay_ms(100);
		pick();
		_delay_ms(100);
		stop();
		right_turn_wls();
		right_turn_wls();
		current_node = 12;
		previous_node = 16;
		dijkstra(graph, 12, 8);
		stop();
		forward_wls();
		_delay_ms(40);
		stop();
		_delay_ms(100);
		place();
		_delay_ms(100);
		stop();
		right_turn_wls();
		current_node = 7;
		previous_node = 8;
		dijkstra(graph, 7, 17);
		if (previous_node == 16)
		{
			forward_wls();
			forward();
			_delay_ms(50);
			left_turn_wls();
			stop();
			left();
		}
		else if (previous_node == 18)
		{
			forward_wls();
			forward();
			_delay_ms(50);
			right_turn_wls();
			stop();
			right();
		}
		else if (previous_node == 13)
		{
			forward_wls();
			_delay_ms(50);
			stop();
			_delay_ms(100);
		}
	}
	else if (isGreen())
	{
		green++;
		_delay_ms(100);
		pick();
		_delay_ms(100);
		stop();
		right_turn_wls();
		right_turn_wls();
		current_node = 12;
		previous_node = 16;
		dijkstra(graph, 12, 11);
		stop();
		forward_wls();
		_delay_ms(40);
		stop();
		_delay_ms(100);
		place();
		_delay_ms(100);
		stop();
		right_turn_wls();
		current_node = 10;
		previous_node = 11;
		dijkstra(graph, 10, 17);
		if (previous_node == 16)
		{
			forward_wls();
			forward();
			_delay_ms(50);
			left_turn_wls();
			stop();
			left();
		}
		else if (previous_node == 18)
		{
			forward_wls();
			forward();
			_delay_ms(50);
			right_turn_wls();
			stop();
			right();
		}
		else if (previous_node == 13)
		{
			forward_wls();
			_delay_ms(50);
			stop();
			_delay_ms(100);
		}
	}
	if (isRed())
	{
		if (red == 0)
		{
			red++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			right_turn_wls();
			right_turn_wls();
			current_node = 13;
			previous_node = 17;
			dijkstra(graph, 13, 8);
			stop();
			forward_wls();
			_delay_ms(40);
			stop();
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 7;
			previous_node = 8;
			dijkstra(graph, 7, 18);
			if (previous_node == 17)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				left_turn_wls();
				stop();
				left();
			}
			else if (previous_node == 19)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				right_turn_wls();
				stop();
				right();
			}
		}
		else if (red == 1)
		{
			red++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			right_turn_wls();
			right_turn_wls();
			current_node = 13;
			previous_node = 17;
			dijkstra(graph, 13, 6);
			stop();
			if (previous_node == 7)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				right_turn_wls();
				stop();
				right();
			}
			else if (previous_node == 3)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				left_turn_wls();
				stop();
				left();
			}
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			previous_node = 6;
			current_node = 7;
			dijkstra(graph, 7, 18);
			if (previous_node == 17)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				left_turn_wls();
				stop();
				left();
			}
			else if (previous_node == 19)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				right_turn_wls();
				stop();
				right();
			}

		}
	}
	if (isGreen())
	{
		if (green == 0)
		{
			green++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			right_turn_wls();
			right_turn_wls();
			current_node = 13;
			previous_node = 17;
			dijkstra(graph, 13, 11);
			stop();
			forward_wls();
			_delay_ms(40);
			stop();
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 10;
			previous_node = 11;
			dijkstra(graph, 10, 18);
			if (previous_node == 17)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				left_turn_wls();
				stop();
				left();
			}
			else if (previous_node == 19)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				right_turn_wls();
				stop();
				right();
			}
		}
		else if (green == 1)
		{
			green++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			right_turn_wls();
			right_turn_wls();
			current_node = 13;
			previous_node = 17;
			dijkstra(graph, 13, 9);
			stop();
			if (previous_node == 5)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				right_turn_wls();
				stop();
				right();
			}
			else if (previous_node == 10)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				left_turn_wls();
				stop();
				left();
			}
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			previous_node = 9;
			current_node = 5;
			dijkstra(graph, 5, 18);
			if (previous_node == 17)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				left_turn_wls();
				stop();
				left();
			}
			else if (previous_node == 19)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				right_turn_wls();
				stop();
				right();
			}

		}
	}
	if (isRed())
	{
		if (red == 0)
		{
			red++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 19;
			previous_node = 18;
			dijkstra(graph, 19, 8);
			stop();
			forward_wls();
			_delay_ms(40);
			stop();
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 7;
			previous_node = 8;
			dijkstra(graph, 7, 20);
			if (previous_node == 19)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				left_turn_wls();
				stop();
				left();
			}
			else if (previous_node == 21)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				right_turn_wls();
				stop();
				right();
			}
		}
		else if (red == 1)
		{
			red++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 19;
			previous_node = 18;
			dijkstra(graph, 19, 6);
			stop();
			if (previous_node == 7)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				right_turn_wls();
				stop();
				right();
			}
			else if (previous_node == 3)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				left_turn_wls();
				stop();
				left();
			}
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			previous_node = 6;
			current_node = 7;
			dijkstra(graph, 7, 20);
			if (previous_node == 19)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				left_turn_wls();
				stop();
				left();
			}
			else if (previous_node == 21)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				right_turn_wls();
				stop();
				right();
			}

		}
	}
	if (isGreen())
	{
		if (green == 0)
		{
			green++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 19;
			previous_node = 18;
			dijkstra(graph, 19, 11);
			stop();
			forward_wls();
			_delay_ms(40);
			stop();
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 10;
			previous_node = 11;
			dijkstra(graph, 10, 20);
			if (previous_node == 19)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				left_turn_wls();
				stop();
				left();
			}
			else if (previous_node == 21)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				right_turn_wls();
				stop();
				right();
			}
		}
		else if (green == 1)
		{
			green++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 19;
			previous_node = 18;
			dijkstra(graph, 19, 9);
			stop();
			if (previous_node == 5)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				right_turn_wls();
				stop();
				right();
			}
			else if (previous_node == 10)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				left_turn_wls();
				stop();
				left();
			}
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			previous_node = 9;
			current_node = 5;
			dijkstra(graph, 5, 20);
			if (previous_node == 19)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				left_turn_wls();
				stop();
				left();
			}
			else if (previous_node == 21)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				right_turn_wls();
				stop();
				right();
			}
		}
	}
	if (isRed())
	{

		if (red == 0)
		{
			red++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 21;
			previous_node = 20;
			dijkstra(graph, 21, 8);
			stop();
			forward_wls();
			_delay_ms(40);
			stop();
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 7;
			previous_node = 8;
			dijkstra(graph, 7, 21);
			if (previous_node == 20)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				left_turn_wls();
				stop();
				left();
			}
			else if (previous_node == 22)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				right_turn_wls();
				stop();
				right();
			}
			else if (previous_node == 15)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				stop();
			}
		}
		else if (red == 1)
		{
			red++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 21;
			previous_node = 20;
			dijkstra(graph, 21, 6);
			stop();
			if (previous_node == 7)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				right_turn_wls();
				stop();
				right();
			}
			else if (previous_node == 3)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				left_turn_wls();
				stop();
				left();
			}
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			previous_node = 6;
			current_node = 7;
			dijkstra(graph, 7, 21);
			if (previous_node == 20)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				left_turn_wls();
				stop();
				left();
			}
			else if (previous_node == 22)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				right_turn_wls();
				stop();
				right();
			}
			else if (previous_node == 15)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				stop();
			}
		}
	}
	if (isGreen())
	{
		if (green == 0)
		{
			green++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 21;
			previous_node = 20;
			dijkstra(graph, 21, 11);
			stop();
			forward_wls();
			_delay_ms(40);
			stop();
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 10;
			previous_node = 11;
			dijkstra(graph, 10, 21);
			if (previous_node == 20)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				left_turn_wls();
				stop();
				left();
			}
			else if (previous_node == 22)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				right_turn_wls();
				stop();
				right();
			}
			else if (previous_node == 15)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				stop();
			}
		}
		else if (green == 1)
		{
			green++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 21;
			previous_node = 20;
			dijkstra(graph, 21, 9);
			stop();
			if (previous_node == 5)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				right_turn_wls();
				stop();
				right();
			}
			else if (previous_node == 10)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				left_turn_wls();
				stop();
				left();
			}
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			previous_node = 9;
			current_node = 5;
			dijkstra(graph, 5, 21);
			if (previous_node == 20)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				left_turn_wls();
				stop();
				left();
			}
			else if (previous_node == 22)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				right_turn_wls();
				stop();
				right();
			}
			else if (previous_node == 15)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				stop();
			}
		}
	}
	if (isRed())
	{
		if (red == 0)
		{
			red++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 22;
			previous_node = 21;
			dijkstra(graph, 22, 8);
			stop();
			forward_wls();
			_delay_ms(40);
			stop();
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 7;
			previous_node = 8;
			dijkstra(graph, 7, 22);
			if (previous_node == 21)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				left_turn_wls();
				stop();
				left();
			}
			else if (previous_node == 14)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				stop();
			}
		}
		else if (red == 1)
		{
			red++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 22;
			previous_node = 21;
			dijkstra(graph, 21, 6);
			stop();
			if (previous_node == 7)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				right_turn_wls();
				stop();
				right();
			}
			else if (previous_node == 3)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				left_turn_wls();
				stop();
				left();
			}
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			previous_node = 6;
			current_node = 7;
			dijkstra(graph, 7, 22);
			if (previous_node == 21)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				left_turn_wls();
				stop();
				left();
			}
			else if (previous_node == 14)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				stop();
			}
		}
	}
	if (isGreen())
	{
		if (green == 0)
		{
			green++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 22;
			previous_node = 21;
			dijkstra(graph, 22, 11);
			stop();
			forward_wls();
			_delay_ms(40);
			stop();
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 10;
			previous_node = 11;
			dijkstra(graph, 10, 22);
			if (previous_node == 21)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				left_turn_wls();
				stop();
				left();
			}
			else if (previous_node == 14)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				stop();
			}
		}
		else if (green == 1)
		{
			green++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			right_turn_wls();
			current_node = 22;
			previous_node = 21;
			dijkstra(graph, 21, 9);
			stop();
			if (previous_node == 5)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				right_turn_wls();
				stop();
				right();
			}
			else if (previous_node == 10)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				left_turn_wls();
				stop();
				left();
			}
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			previous_node = 9;
			current_node = 5;
			dijkstra(graph, 5, 22);
			if (previous_node == 21)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				left_turn_wls();
				stop();
				left();
			}
			else if (previous_node == 14)
			{
				forward_wls();
				forward();
				_delay_ms(50);
				stop();
			}
		}
	}

	if (isRed())
	{
		if (red == 0)
		{
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			left_turn_wls();
			left_turn_wls();
			current_node = 14;
			previous_node = 22;
			dijkstra(graph, 14, 8);
			stop();
			forward_wls();
			_delay_ms(40);
			stop();
			_delay_ms(100);
			place();
			_delay_ms(100);
			right_turn_wls();
			previous_node = 8;
			current_node = 7;
			dijkstra(graph, 7, 0);
			forward_wls();
			left_turn_wls();
			left_turn_wls();
			stop();
		}
		else if (red == 1)
		{
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			left_turn_wls();
			left_turn_wls();
			current_node = 14;
			previous_node = 22;
			dijkstra(graph, 14, 6);
			stop();
			if (previous_node == 7)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				right_turn_wls();
				stop();
				right();
			}
			else if (previous_node == 3)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				left_turn_wls();
				stop();
				left();
			}
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			previous_node = 6;
			current_node = 7;
			dijkstra(graph, 7, 0);
			forward_wls();
			left_turn_wls();
			left_turn_wls();
			stop();
		}
	}
	if (isGreen())
	{
		if (green == 0)
		{
			red++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			left_turn_wls();
			left_turn_wls();
			current_node = 14;
			previous_node = 22;
			dijkstra(graph, 14, 11);
			stop();
			forward_wls();
			_delay_ms(40);
			stop();
			_delay_ms(100);
			place();
			_delay_ms(100);
			right_turn_wls();
			previous_node = 11;
			current_node = 10;
			dijkstra(graph, 10, 0);
			forward_wls();
			left_turn_wls();
			left_turn_wls();
			stop();
		}
		else if (green == 1)
		{
			red++;
			_delay_ms(100);
			pick();
			_delay_ms(100);
			stop();
			left_turn_wls();
			left_turn_wls();
			current_node = 14;
			previous_node = 22;
			dijkstra(graph, 14, 9);
			stop();
			if (previous_node == 5)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				right_turn_wls();
				stop();
				right();
			}
			else if (previous_node == 10)
			{
				forward_wls();
				stop();
				_delay_ms(100);
				left_turn_wls();
				stop();
				left();
			}
			_delay_ms(100);
			place();
			_delay_ms(100);
			stop();
			right_turn_wls();
			previous_node = 9;
			current_node = 5;
			dijkstra(graph, 5, 0);
			forward_wls();
			left_turn_wls();
			left_turn_wls();
			stop();
		}
	}
}