#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>//pthreads
#include <signal.h>

#include <ros/ros.h>

#include "checkboard_navigation_module.h"
#include "data_structure.hpp"
#include <geometry_msgs/Pose2D.h>

using namespace std;



//Control C handler
void my_handler(int s) {
    cout << "Caught signal "<< s <<"\n";
    exit(1);
}

volatile bool stop_flag = false;

void got_theta_IMU(std_msgs::Float32 theta)
{
    D.imu_theta=theta.data;
}

ros::Publisher right_track_pub;
ros::Publisher left_track_pub; 

void got_target_pos(geometry_msgs::Pose2D target_pos)
{
    //Actually process path
    std_msgs::Float32 t_right;
    t_right.data = v_right;
    right_track_pub.publish(t_right);
    std_msgs::Float32 t_left;
    t_left.data = v_left;
    left_track_pub.publish(t_left);
}

int main(int argc, char **argv)
{

    //control C handling
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    //ROS init
    ros::init(argc, argv, "IRIS_Navigation_and_planning", ros::init_options::NoSigintHandler);

    pthread_t chessboard_t;
    pthread_t navigation_t;

    

    int chessboard = pthread_create(&chessboard_t, NULL, init_chessboard_navigation, (void*)&stop_flag);
    int navigation = pthread_create(&navigation_t, NULL, init_kinect_mapping, (void*)&stop_flag);
    if(navigation || chessboard )
        exit(EXIT_FAILURE);

    ros::NodeHandle n("main_estimator");
    ros::Subscriber sub = n.subscribe("/IRIS/theta_IMU", 1, got_theta_IMU);
    ros::Subscriber sub = n.subscribe("/IRIS/target_pose", 1,got_target_pos);
    ros::Publisher current_pos_pub pub = n.advertise<geometry_msgs::Pose2D>("/IRIS/current_pose", 1);
    right_track_pub = n.advertise<std_msgs::Float32>("/IRIS/webcam_angle", 1);
    left_track_pub = n.advertise<std_msgs::Float32>("/IRIS/webcam_angle", 1);

    ros::Rate r(50);
    while (ros::ok())
    {
        //do the actual estimator
        geometry_msgs::Pose2D current_pos;
        current_pos.x=D.true_pos_x;
        current_pos.y=D.true_pos_y;
        current_pos.theta=D.true_theta;
        current_pos_pub.publish(current_pos);
        ros::spinOnce();
        r.sleep();
    }

    stop_flag = true;

    pthread_join(chessboard_t, 0);
    pthread_join(navigation_t, 0);


}
