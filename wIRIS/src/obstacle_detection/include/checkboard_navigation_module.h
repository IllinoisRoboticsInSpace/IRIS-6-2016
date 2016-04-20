//Checkboard navigation header

struct chesspos{
float x,y;
time_t time;
};

chesspos get_chessboard_navigation_pos();

int init_chessboard_navigation(const std::string inputSettingsFile, volatile bool * stop_flag );

