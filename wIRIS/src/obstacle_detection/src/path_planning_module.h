#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include "checkboard_navigation_module.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>

using namespace std;
using namespace cv;

#define pow2(x) (x)*(x)

#define LINEAR_CONST  2
#define ANGULAR_CONST 0.2

//Global variables
double goal_x;
double goal_y;

double forward_cntl;
double turning_cntl;

void path_planning(void* a)
{
	while(1)
	{
		//Get robot position
		chesspos pos;
		pos = get_chessboard_navigation_pos();

		//Message setup
		forward_cntl = sqrt(pow2(goal_x - pos.x) + pow2(goal_y - pos.y))/LINEAR_CONST;
		turning_cntl = (atan2(goal_y - pos.y, goal_x - pos.x) - pos.t)/ANGULAR_CONST;
	}
}

void FSM_callback()
{
	//States
	enum state {wait_to_start, localize, move_to_mine, mine, move_to_deposit, deposit, error};

}