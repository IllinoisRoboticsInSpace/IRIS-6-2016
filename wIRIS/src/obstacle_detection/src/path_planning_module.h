#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "checkboard_navigation_module.h"
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>

using namespace std;
using namespace cv;

#define pow2(x) (x)*(x)

const double LINEAR_CONST = 1/2.;
const double ANGULAR_CONST = 1/0.2;

//Global variables
volatile double goal_x;
volatile double goal_y;

enum{RETRACT=0;STAY=1;EXTEND=2;STOP=0;MOVE=1;BACKWARDS=-2};

volatile int bin_movement = 1; // 0=RETRACT 1=STAY 2=EXTEND
volatile int paddle_movement = 1; // 0=RETRACT 1=STAY 2=EXTEND
volatile int paddle_onoff = 0; // 0=STOP 1=MOVE


volatile int control_direction=1;

//absolute value templated
template<typename T> T absd(T d){return d<0?-d:d;}
//efficient square templated
template<typename T> T pow2(T d){return d*d;}
//find distance in S(2pi) set (like a circle: through either side)
template<typename T> T diff2pi(T d)
{
	d=fmod2pi(d);
	return min(d,2*M_PI-d);}
}

void path_planning(void* a)
{
	//Initialize ROS node and publisher
	ros::NodeHandle n;
	ros::Publisher pub_control=n.advertise<std_msgs::String>("/IRIS/autonomous_command" "hh", 1);
	while(1)
	{
		double forward_cntl;
		double turning_cntl;
		//Get robot position
		chesspos pos = get_chessboard_navigation_pos();

		//Message setup
		if(control_direction==BACKWARDS)
		{
			forward_cntl=-1000;
			turning_cntl = 0.;
		}
		else
		{
			forward_cntl = control_direction*sqrt(pow2(goal_x - pos.x) + pow2(goal_y - pos.y))*LINEAR_CONST;
			if(control_direction>0)
				turning_cntl = diff2pi(fmod2pi(atan2(goal_y - pos.y, goal_x - pos.x)) - fmod2pi(pos.t))*ANGULAR_CONST;
			else if(control_direction<0)
				turning_cntl = diff2pi(fmod2pi(atan2(goal_y - pos.y, goal_x - pos.x)) - fmod2pi(pos.t+M_PI))*ANGULAR_CONST;
			else
				turning_cntl = 0.;
		}
		
		//normalize and get right and left values
		double normalizer=absd(turning_cntl)+absd(forward_cntl);
		turning_cntl/=normalizer;
		forward_cntl/=normalizer;
		double right=forward_cntl+turning_cntl;
		double left=forward_cntl-turning_cntl;

		//publish messages
		std_msgs::String msg;
		std::stringstream ss;
		ss << left << "," << right << "," << bin_movement << "," << paddle_movement << "," << paddle_onoff ;
		msg.data = ss.str();
		pub_control.publish(msg);

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
