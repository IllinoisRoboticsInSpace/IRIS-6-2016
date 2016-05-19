#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <unistd.h>

#include "checkboard_navigation_module.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>

using namespace std;
using namespace cv;

#define pow2(x) (x)*(x)

#define LINEAR_CONST  2
#define ANGULAR_CONST 0.2

//Global variables
volatile double goal_x;
volatile double goal_y;

extern volatile double forward_cntl;
extern volatile double turning_cntl;

//States
enum state_t {wait_to_start, localize, move_to_mine, mine, move_to_deposit, deposit, error};
int cur_state;

void path_planning(void* a)
{
	while(1)
	{
		//Get robot position
		chesspos pos = get_chessboard_navigation_pos();

		//Message setup
		forward_cntl = sqrt(pow2(goal_x - pos.x) + pow2(goal_y - pos.y))/LINEAR_CONST;
		turning_cntl = (atan2(goal_y - pos.y, goal_x - pos.x) - pos.t)/ANGULAR_CONST;
	}
}

//Sets a goal to move to
void set_goal(double x, double y, int dir)
{
	//Set the goal position to the x and y values
	goal_x = x;
	goal_y = y;
	control_direction = dir;
}

//Waits until robot reaches destination within specified tolerance
void wait_for_dist(double epsilon)
{
	//Wait in here until the robot position is at or within the allowed tolerance
	chesspos pos = get_chessboard_navigation_pos();
	double dist = sqrt(pow2(goal_x - pos.x) + pow2(goal_y - pos.y));

	while(dist > epsilon)
	{
		//Recompute distance from goal
		pos = get_chessboard_navigation_pos();
		dist = sqrt(pow2(goal_x - pos.x) + pow2(goal_y - pos.y));

		//Wait for 10 ms
		sleep(0.01);
	}

	control_direction = 0;
}

//Main Finite State Machine
void FSM()
{
	//Sequentially move through the different states: move_to_mine -> mine -> move_to_deposit -> deposit
	//Offset for varying the x-axis position of the goal states and iteration level
	int offset[3] = {0, 100, -100};
	int iter = 0;

	double x;
	double y;
	double epsilon = 100;

	while(1)
	{
		//Move to mine
		x = offset[iter];
		y = 500;
		//epsilon = 0.2*y;
		set_goal(x, y, 1);
		wait_for_dist(epsilon);

		//Mine
		//Order: start Maxon -> lower paddle -> set_goal() -> wait_for_dist() -> raise paddle -> stop Maxon
		x = offset[iter];
		y += 50;
		//epsilon = 0.2*y;
		
		paddle_onoff = MOVE;
		paddle_movement = RETRACT;
		set_goal(x, y, 1);
		sleep(10); //???
		paddle_movement = STAY
		wait_for_dist(epsilon);
		paddle_movement = EXTEND;
		sleep(10); //???
		paddle_onoff = STOP;
		paddle_movement = STAY;

		//Move to deposit
		//Align to center of arena
		x = 0;
		y = 297;
		//epsilon = 0.2*y;
		set_goal(x, y, -1);
		wait_for_dist(epsilon);

		//Move up to bin
		x = 0;
		y = -50;
		//epsilon = 100;
		set_goal(x, y, -1);
		wait_for_dist(epsilon);

		//Now just move straight back until we reack the collection bin
		control_direction = BACKWARDS;
		sleep(10); //???

		//Deposit
		bin_movement = EXTEND;
		sleep(15) //~15s
		bin_movement = STAY;
		sleep(5); //???
		bin_movement = RETRACT;
		sleep(10); //~10-15s
		bin_movement = STAY;

		//Increment the iteration
		iter = (iter + 1) % 3;
	}
}