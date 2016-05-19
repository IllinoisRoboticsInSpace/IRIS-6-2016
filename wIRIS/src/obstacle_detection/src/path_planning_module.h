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

void set_goal(double x, double y)
{
	//Set the goal position to the x and y values
	goal_x = x;
	goal_y = y;
}

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
}

void FSM()
{
	//Sequentially move through the different states: move_to_mine -> mine -> move_to_deposit -> deposit
	//Offset for varying the x-axis position of the goal states and iteration level
	int offset[3] = {0, 100, -100};
	int iter = 0;

	double x;
	double y;
	double epsilon;

	while(1)
	{
		//Move to mine
		x = offset[iter];
		y = 434;
		epsilon = 0.2*y;
		set_goal(x, y);
		wait_for_dist(epsilon);

		//Mine
		//mine(true)
		y += 50;
		epsilon = 0.2*y;
		set_goal(x, y);
		wait_for_dist(epsilon);
		//mine(false)

		//Move to deposit
		x = 0;
		y = 297;
		epsilon = 0.2*y;
		set_goal(x, y);
		wait_for_dist(epsilon);

		//Deposit
		//deposit(...)

		//Increment the iteration
		iter = (iter + 1) % 3;
	}
}