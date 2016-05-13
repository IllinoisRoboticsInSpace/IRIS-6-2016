#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "checkboard_navigation_module.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;

#include "claibinit_mod.h"

double fmod2pi(double v)
{
    return fmod(fmod(v,M_PI*2)+M_PI*4,M_PI*2);
}

long int millis()
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    return tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
}


//Global settings file
chesspos pos_chesspos={0.,0.,0.,0};
int lock=0;

chesspos get_chessboard_navigation_pos()
{
    while(lock)cout<<"Process is locked trying to access chessboard navigation data"<<endl;
    lock=1;
    chesspos ret;
    ret.x=pos_chesspos.x;
    ret.y=pos_chesspos.y;
    ret.t=pos_chesspos.t;
    ret.millis=pos_chesspos.millis;
    lock=0;
    return ret;
}


long convertToEpoch(long v4l_ts_ms) {
    //To solve any issues go to:
    //    http://answers.opencv.org/question/61099/is-it-possible-to-get-frame-timestamps-for-live-streaming-video-frames-on-linux/
    static long monotonicToEpochOffset_ms = -1; //<-- initialize in global scope
    if (monotonicToEpochOffset_ms == -1) {
        struct timespec  vsTime;  clock_gettime(CLOCK_MONOTONIC, &vsTime);

        long uptime_ms = vsTime.tv_sec * 1000 + (long)round(vsTime.tv_nsec / 1000000.0);
        long epoch_ms = millis();

        // add this quantity to the CV_CAP_PROP_POS_MEC to get unix time stamped frames
        monotonicToEpochOffset_ms = epoch_ms - uptime_ms;
    }

    return monotonicToEpochOffset_ms + v4l_ts_ms;

}

bool nextImage(VideoCapture & inputCapture, Mat & result, long * millis_timestamp=0)
{
    for (long i = millis();millis() - i < 20;)//stay 20 ms discarding frames to ensure we get a current frame at the end
    {
        long j = millis();
        if (!inputCapture.grab())
        {
            cout << "WEBCAM IMAGE GRAB FAILED eventually trying to restart\n";
            return false;
        }
        if (millis() - j > 10)
            break;
    }
    inputCapture.retrieve(result);

    //cout<<"CV_CAP_PROP_POS_MSEC: " << convertToEpoch(inputCapture.get(CV_CAP_PROP_POS_MEC)) <<"\n" <<
          //"CV_CAP_PROP_POS_FRAMES: "<< (long)inputCapture.get(CV_CAP_PROP_POS_FRAMES) << "\n" <<  // <-- the v4l2 'sequence' field
          //"CV_CAP_PROP_FPS:  "<< inputCapture.get(CV_CAP_PROP_FPS) << "\n";

    if (millis_timestamp)*millis_timestamp = millis(); //convertToEpoch(inputCapture.get(CV_CAP_PROP_POS_MEC));

    return true;
}

void* init_chessboard_navigation(void * stop_flag_ptr )
{
    volatile bool * stop_flag = (bool*) stop_flag_ptr;
    ros::NodeHandle n("chessboard_navigation");
    ros::Publisher pub = n.advertise<std_msgs::Float32>("/IRIS/webcam_angle", 1);
    float webcam_angle = 0;
    pos_chesspos.x = 0;
    pos_chesspos.y = 0;
    pos_chesspos.t = 0;
    pos_chesspos.millis = 0;
    Size boardSize(4,3);   //CHANGE HERE THE AMOUNT OF SQUARES
    float depth = 6.03;    //CHANGE HERE THE SHAPE  OF SQUARES
    float squareSize =4.13;//CHANGE HERE THE SHAPE  OF SQUARES

    int camera_id = 0;

    for (int re_connect_retries = 0;!(*stop_flag);++re_connect_retries)
    {
        cout << "Initializing webcam navigation with device " << camera_id << endl;

        VideoCapture inputCapture; 
        inputCapture.open(camera_id); //OPEN CAMERA camera_id
        if (!inputCapture.isOpened())
        {
            camera_id++;
            if (camera_id > 64)
            {
                sleep(0.100);
                camera_id = 0;
            }
            continue; // try next device
        }

        inputCapture.set(CV_CAP_PROP_FRAME_WIDTH, 6400);
        inputCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 4800);
        inputCapture.set(CV_CAP_PROP_FPS, 15);

        Mat cameraMatrix, distCoeffs;
        Size imageSize;
        clock_t prevTimestamp = 0;
        const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
        const char ESC_KEY = 27;

        cout << "Webcam navigation ready!" << endl;

        int count_lost = 0;

        for (int i = 0;!(*stop_flag);++i)
        {
            long millis_timestamp;

            Mat view;
            if (!nextImage(inputCapture, view,&millis_timestamp))
                break;
            if (i == 0) cout << "webcam image size is " << view.cols << "x" << view.rows << "\n";

            //cout << "Webcam navigation data!" << endl;

            imageSize = view.size();  // Format input image.
            if (imageSize.width == 0)
            {
                cout << "Empty webcam image received :(  *************************" << endl;
                static int count_fail = 10;
                sleep(0.010);
                if (count_fail-- <= 0)
                {
                    count_fail = 10;
                    break;
                }
                continue;
            }
            flip(view, view, 0);

            vector<Point2f> pointBuf;

            bool found = findChessboardCorners(view, boardSize
                    pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

            if (found)                // If done with success,
            {
                count_lost = 0;
                // improve the found corners' coordinate accuracy for chessboard
                Mat viewGray;
                cvtColor(view, viewGray, COLOR_BGR2GRAY);
                cornerSubPix(viewGray, pointBuf, Size(11, 11),
                    Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                

                // Draw the corners.
                drawChessboardCorners(view, boardSize, Mat(pointBuf), found);

                //cout << endl<<endl<< "***************** list *******************" << endl;

                //if (boardSize.width == 4 && boardSize.height == 3)
                {
                    const int Alist[16][2] = {
                        { 0, 4},{ 0, 5},
                        { 1, 5},{ 1, 4},
                        { 2, 6},{ 2, 7},
                        { 3, 7},{ 3, 6},
                        { 4, 8},{ 4, 9},
                        { 5, 9},{ 5, 8},
                        { 6,10},{ 6,11},
                        { 7,11},{ 7,10},
                    };
                    const int Blist[12][2] = {
                        { 0, 2},{ 0, 3},
                        { 1, 3},{ 1, 2},
                        { 4, 6},{ 4, 7},
                        { 5, 7},{ 5, 6},
                        { 8,10},{ 8,11},
                        { 9,11},{ 9,10},
                    };
                    float a = 0;
                    float b = 0;
                    float c = 0;
                    float& w = squareSize;
                    float& d = depth;
                    for (int ii = 0;ii < 16;ii++)
                        a += pointBuf[Alist[ii][0]].x - pointBuf[Alist[ii][1]].x;
                    for (int ii = 0;ii < 12;ii++)
                        b += pointBuf[Blist[ii][0]].x - pointBuf[Blist[ii][1]].x;
                    for (int ii = 0;ii < 12;ii++)
                        c += pointBuf[Blist[ii][0]].x;
                    a /= 16;
                    b /= 12;
                    c /= 12;
                    if (a < 0)
                    {
                        a = -a;
                        b = -b;
                    }
                    //find circles center and radious
                    //for(vector<Point2f>::iterator it=pointBuf.begin();it!=pointBuf.end();it++)
                    //{
                    //    cout << "Point " << it->x << " , " << it->y << endl;
                    //}
                    //cout << "a " << a/50 << " b " << b/50 << endl;
                    a *= 2.226618 / view.cols;
                    b *= 2.226618 / view.cols;
                    c -= view.cols / 2;
                    c *= 2.226618 / view.cols;
                    float dx = w + d*tan(M_PI / 2 - b);
                    float dy = d + w*tan(M_PI / 2 - a);
                    float dperp = (w*dy - (dy * 2 - d)*dx) / (dx*dx + dy*dy);
                    float x = w + dy*dperp;
                    float y = d - dx*dperp;
                    //cout << "dx " << dx << " dy " << dy << endl;

                    //rotate webcam!
                    float delta = c*180. / M_PI / 4.;
                    if (abs(delta) > 1)
                        webcam_angle += delta;
                    bool long_turn = false;
                    if (webcam_angle < 0)
                    {
                        webcam_angle = 355;
                        long_turn = true;
                    }
                    if (webcam_angle > 360)
                    {
                        webcam_angle = 5;
                        long_turn = true;
                    }
                    std_msgs::Float32 msg;
                    msg.data = webcam_angle;
                    pub.publish(msg);

                    double vehicle_angle = fmod2pi(webcam_angle*M_PI / 180. - atan2(y, x) - M_PI);

                    cout << "webcam nav x " << x << " y " << y << " th " << webcam_angle << " delta " << delta << " vehicle " << vehicle_angle*180. / M_PI << endl;

                    while (lock);
                    lock = 1;
                    pos_chesspos.x = x;
                    pos_chesspos.y = y;
                    pos_chesspos.t = vehicle_angle;
                    pos_chesspos.millis = millis_timestamp;
                    lock = 0;

                    if (long_turn || abs(delta) > 1)
                    {
                        long int t = millis();
                        while (millis() - t < (long_turn ? 3000 : 200))
                             nextImage(inputCapture, view);
                    }

                }
                else cout << "Wrong pattern size!";
            }
            else
            {
                count_lost++;
                if (count_lost > 10)
                {
                    cout << "WEBCAM: too long lost, doing 360s\n";
                    static int sweep_dir = 1;

                    static const int delta_angle = 45;

                    if (webcam_angle + delta_angle*sweep_dir > 360 || webcam_angle + delta_angle*sweep_dir < 0)
                        sweep_dir = -sweep_dir;
                    webcam_angle += delta_angle*sweep_dir;
                    std_msgs::Float32 msg;
                    msg.data = webcam_angle;
                    pub.publish(msg);

                    long int t = millis();
                    while (millis() - t < 600)
                         nextImage(inputCapture, view);
                    count_lost = 8;
                }
                else
                    cout << "WEBCAM: pattern lost\n";
            }

        }

        cout << "WEBCAM ERROR - Lost connection to camera!";

    }
    cout << "*** Webcam process finished";

    return 0;
}


