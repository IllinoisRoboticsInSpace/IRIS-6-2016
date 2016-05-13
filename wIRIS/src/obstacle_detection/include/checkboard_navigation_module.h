//Checkboard navigation header

#include <string>

struct chesspos{
    float x,y;
    float t;
    long int millis;
};

chesspos get_chessboard_navigation_pos();

int init_chessboard_navigation( void * stop_flag_ptr );

long int millis();
double fmod2pi(double v);

int init_kinect_mapping(void * stop_flag);

