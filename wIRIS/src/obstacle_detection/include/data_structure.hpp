// Main data structure
#include "Map.hpp"//Map<T>



class navigation_and_mapping_data {
    volatile MATRIX* map;
    volatile MATRIX* local_map;
    volatile double true_pos_x;
    volatile double true_pos_y;
    volatile double true_theta;
    volatile double imu_theta;
    volatile double track_right;
    volatile double track_left;
};

chesspos get_chessboard_navigation_pos();

extern navigation_and_mapping_data D={0,0};
