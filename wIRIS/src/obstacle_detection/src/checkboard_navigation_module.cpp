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


#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;

static void help()
{
    cout <<  "This is a camera calibration sample." << endl
         <<  "Usage: calibration configurationFile"  << endl
         <<  "Near the sample file you'll find the configuration file, which has detailed help of "
             "how to edit it.  It may be any OpenCV supported file format XML/YAML." << endl;
}
class Settings
{
public:
    Settings() : goodInput(false) {}
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType {INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST};

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{" << "BoardSize_Width"  << boardSize.width
                  << "BoardSize_Height" << boardSize.height
                  << "BoardSize_Depth" << depth
                  << "Square_Size"         << squareSize
                  << "Calibrate_Pattern" << patternToUse
                  << "Calibrate_NrOfFrameToUse" << nrFrames
                  << "Calibrate_FixAspectRatio" << aspectRatio
                  << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
                  << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

                  << "Write_DetectedFeaturePoints" << bwritePoints
                  << "Write_extrinsicParameters"   << bwriteExtrinsics
                  << "Write_outputFileName"  << outputFileName

                  << "Show_UndistortedImage" << showUndistorsed

                  << "Input_FlipAroundHorizontalAxis" << flipVertical
                  << "Input_Delay" << delay
                  << "Input" << input
           << "}";
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["BoardSize_Width" ] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["BoardSize_Depth"] >> depth;
        node["Calibrate_Pattern"] >> patternToUse;
        node["Square_Size"]  >> squareSize;
        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
        node["Calibrate_FixAspectRatio"] >> aspectRatio;
        node["Write_DetectedFeaturePoints"] >> bwritePoints;
        node["Write_extrinsicParameters"] >> bwriteExtrinsics;
        node["Write_outputFileName"] >> outputFileName;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
        node["Show_UndistortedImage"] >> showUndistorsed;
        node["Input"] >> input;
        node["Input_Delay"] >> delay;
        interprate();
    }
    void interprate()
    {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
        if (nrFrames <= 0)
        {
            cerr << "Invalid number of frames " << nrFrames << endl;
            goodInput = false;
        }

        if (input.empty())      // Check for valid input
                inputType = INVALID;
        else
        {
            if (input[0] >= '0' && input[0] <= '9')
            {
                stringstream ss(input);
                ss >> cameraID;
                inputType = CAMERA;
            }
            else
            {
                if (readStringList(input, imageList))
                    {
                        inputType = IMAGE_LIST;
                        nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
                    }
                else
                    inputType = VIDEO_FILE;
            }
            if (inputType == CAMERA)
            {
                inputCapture.open(cameraID);
                inputCapture.set(CV_CAP_PROP_FRAME_WIDTH,6400);
                inputCapture.set(CV_CAP_PROP_FRAME_HEIGHT,4800);
                inputCapture.set(CV_CAP_PROP_FPS,15);
            }
            if (inputType == VIDEO_FILE)
                inputCapture.open(input);
            if (inputType != IMAGE_LIST && !inputCapture.isOpened())
                    inputType = INVALID;
        }
        if (inputType == INVALID)
        {
            cerr << " Inexistent input: " << input;
            goodInput = false;
        }

        flag = 0;
        if(calibFixPrincipalPoint) flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
        if(calibZeroTangentDist)   flag |= CV_CALIB_ZERO_TANGENT_DIST;
        if(aspectRatio)            flag |= CV_CALIB_FIX_ASPECT_RATIO;


        calibrationPattern = NOT_EXISTING;
        if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
        if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
        if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
        if (calibrationPattern == NOT_EXISTING)
            {
                cerr << " Inexistent camera calibration mode: " << patternToUse << endl;
                goodInput = false;
            }
        atImageList = 0;

    }
    Mat nextImage()
    {
        Mat result;
        if( inputCapture.isOpened() )
        {
            Mat view0;
            inputCapture >> view0;
            view0.copyTo(result);
        }
        else if( atImageList < (int)imageList.size() )
            result = imread(imageList[atImageList++], CV_LOAD_IMAGE_COLOR);

        return result;
    }

    static bool readStringList( const string& filename, vector<string>& l )
    {
        l.clear();
        FileStorage fs(filename, FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != FileNode::SEQ )
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
            l.push_back((string)*it);
        return true;
    }
public:
    Size boardSize;             // The size of the board -> Number of items by width and height
    Pattern calibrationPattern; // One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;           // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;               // The number of frames to use from the input for calibration
    float aspectRatio;          // The aspect ratio
    int delay;                  // In case of a video input
    bool bwritePoints;          // Write detected feature points
    bool bwriteExtrinsics;      // Write extrinsic parameters
    bool calibZeroTangentDist;  // Assume zero tangential distortion
    bool calibFixPrincipalPoint;// Fix the principal point at the center
    bool flipVertical;          // Flip the captured images around the horizontal axis
    string outputFileName;      // The name of the file where to write
    bool showUndistorsed;       // Show undistorted images after calibration
    string input;               // The input ->
    float depth;                // depth distance between the two pattern planes (bent horizontally at middle row)


    int cameraID;
    vector<string> imageList;
    int atImageList;
    VideoCapture inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;

private:
    string patternToUse;


};

static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2, FOUND = 3, LOCATED = 4 };

//Global settings file
chesspos pos_chesspos;
int lock=0;

chesspos get_chessboard_navigation_pos()
{
    while(lock)cout<<"m"<<endl;
    lock=1;
    chesspos ret;
    ret.x=pos_chesspos.x;
    ret.y=pos_chesspos.y;
    ret.time=pos_chesspos.time;
    lock=0;
    return ret;
}


int init_chessboard_navigation(const string inputSettingsFile, volatile bool * stop_flag )
{
    pos_chesspos.x=0;
    pos_chesspos.y=0;
    pos_chesspos.time=0;
    cout << "Initializing webcam navigation" << endl;
    Settings s;
    //help();
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int mode = DETECTION;
    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;

    cout << "Webcam navigation ready!" << endl;

    for(int i = 0;!(*stop_flag);++i)
    {
        Mat view;
        bool blinkOutput = false;

        view = s.nextImage();

        //cout << "Webcam navigation data!" << endl;

        imageSize = view.size();  // Format input image.
        if( imageSize.width==0)
        {
            cout << "Empty webcam image received :(  *************************"<< endl;
            static int count_fail=10;
            sleep(0.010);
            if(count_fail--<=0)
            {
               break;
               count_fail=10;
            }
            continue;
        }
        if( s.flipVertical )    flip( view, view, 0 );

        vector<Point2f> pointBuf;

        bool found, located=false;
        switch( s.calibrationPattern ) // Find feature points on the input format
        {
        case Settings::CHESSBOARD:
            found = findChessboardCorners( view, s.boardSize, pointBuf,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
            break;
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf );
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
            break;
        default:
            found = false;
            break;
        }
        
        located=false;

        if ( found)                // If done with success,
        {
              // improve the found corners' coordinate accuracy for chessboard
                if( s.calibrationPattern == Settings::CHESSBOARD)
                {
                    Mat viewGray;
                    cvtColor(view, viewGray, COLOR_BGR2GRAY);
                    cornerSubPix( viewGray, pointBuf, Size(11,11),
                        Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
                }

                // Draw the corners.
                drawChessboardCorners( view, s.boardSize, Mat(pointBuf), found );
                
                //cout << endl<<endl<< "***************** list *******************" << endl;
                
                if(s.boardSize.width==4 && s.boardSize.height==3)
                {
                    const int Alist[16][2]={
                        { 0, 4},{ 0, 5},
                        { 1, 5},{ 1, 4},
                        { 2, 6},{ 2, 7},
                        { 3, 7},{ 3, 6},
                        { 4, 8},{ 4, 9},
                        { 5, 9},{ 5, 8},
                        { 6,10},{ 6,11},
                        { 7,11},{ 7,10},
                        };
                    const int Blist[12][2]={
                        { 0, 2},{ 0, 3},
                        { 1, 3},{ 1, 2},
                        { 4, 6},{ 4, 7},
                        { 5, 7},{ 5, 6},
                        { 8,10},{ 8,11},
                        { 9,11},{ 9,10},
                        };
                    float a=0;
                    float b=0;
                    float& w = s.squareSize;
                    float& d = s.depth;
                    for(int ii=0;ii<16;ii++)
                        a+=pointBuf[Alist[ii][0]].x-pointBuf[Alist[ii][1]].x;
                    for(int ii=0;ii<12;ii++)
                        b+=pointBuf[Blist[ii][0]].x-pointBuf[Blist[ii][1]].x;
                    a/=16;
                    b/=12;
                    if (a<0)
                    {
                        a=-a;
                        b=-b;
                    }
                    //find circles center and radious
                    //for(vector<Point2f>::iterator it=pointBuf.begin();it!=pointBuf.end();it++)
                    //{
                    //    cout << "Point " << it->x << " , " << it->y << endl;
                    //}
                    //cout << "a " << a/50 << " b " << b/50 << endl;
                    a/=50.*180./M_PI;
                    b/=50.*180./M_PI;
                    float dx=w/2+d/2*tan(M_PI/2-b);
                    float dy=d/2+w/2*tan(M_PI/2-a);
                    float dperp=(w*dy-(dy*2-d)*dx)/(dx*dx+dy*dy);
                    float x=w/2+dy*dperp;
                    float y=d/2-dx*dperp;
                    //cout << "dx " << dx << " dy " << dy << endl;
                    cout << "webcam nav x " << x << " y " << y << endl;
                    
                    while(lock);
                    lock=1;
                    pos_chesspos.x=x;
                    pos_chesspos.y=y;
                    pos_chesspos.time=time(0);
                    lock=0;
                    
                    
                    if(false){
                        float rx=sqrt(pow(dx-w/2,2)+pow(d/2,2));
                        float ry=sqrt(pow(dy-d/2,2)+pow(w/2,2));
                        //cout << "rx " << rx << " ry " << ry << endl;
                        float ox=500,oy=100;
                        Mat dgrm = Mat::zeros( 1000, 1000, CV_8UC3 );
                        ellipse( dgrm,
                           Point( ox+dx, oy ),
                           Point( rx,rx ),
                           0,
                           0,
                           360,
                           Scalar( 255, 0, 0 ),
                           1,
                           8);
                        ellipse( dgrm,
                           Point( ox, oy+dy ),
                           Point( ry,ry ),
                           0,
                           0,
                           360,
                           Scalar( 255, 0, 0 ),
                           1,
                           8);
                        rectangle( dgrm,
                           Point( ox+w/2,oy+d/2 ),
                           Point( ox-w/2,oy-d/2),
                           Scalar( 0, 255, 255 ),
                           1,
                           8 );
                        imshow("Diagram", dgrm);
                    }
                    located=true;

                }
                else cout << "Wrong pattern size!";
        }

        //----------------------------- Output Text ------------------------------------------------
        string msg = found ? (located ? "Located" : "Pattern not usable") : "Could not find pattern";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);
        
        //if( blinkOutput )
            //bitwise_not(view, view);

        //------------------------------ Show image and check for input commands -------------------
        //flip( view, view, 1 );
        //putText( view, msg, textOrigin, 1, 1, located ?  GREEN : RED);
        //imshow("Image View", view);
        //char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);
        sleep(0.050);
        //if( key  == ESC_KEY )
        //    break;

    }

    cout << "*** Webcam process finished";

    return 0;
}


