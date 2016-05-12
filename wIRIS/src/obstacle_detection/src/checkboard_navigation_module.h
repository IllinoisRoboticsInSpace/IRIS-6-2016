//Checkboard navigation header

struct chesspos{
float x,y;
float t;
long int millis;
};

chesspos get_chessboard_navigation_pos();

int init_chessboard_navigation(const std::string inputSettingsFile, volatile bool * stop_flag );

long int millis();
