#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>//pthreads
#include <signal.h>

#include <ros/ros.h>

#include "checkboard_navigation_module.h"

using namespace std;



//Control C handler
void my_handler(int s) {
    cout << "Caught signal "<< s <<"\n";
    exit(1);
}

volatile bool stop_flag = false;



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

    while (1)
        sleep(10); //just wait

    stop_flag = true;

    pthread_join(chessboard_t, 0);
    pthread_join(navigation_t, 0);


}
