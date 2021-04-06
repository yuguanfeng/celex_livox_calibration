#include <opencv2/opencv.hpp>
#include "Param.h"

using namespace cv;
using namespace std;

namespace calib{

    int Param::DATASET_NUMBER = 0;
    int Param::GRAY_THRESHOLD = 0;
    float Param::POINTCLOUD_THRESHOLD = 0;

    int Param::WIDTH = 0;
    int Param::HEIGHT = 0;

    Mat Param::camera_matrix = Mat(3,3,CV_64FC1);
    Mat Param::distort = Mat(5,1,CV_64FC1);
    Mat Param::MAPX;
    Mat Param::MAPY;

    int Param::IF_EVENT_CAMERA = 0;

    float Param::OFFSET_X = 0;
    float Param::OFFSET_Y = 0;
    float Param::OFFSET_Z = 0;
    float Param::ANGLE_X = 0;
    float Param::ANGLE_Y = 0;
    float Param::ANGLE_Z = 0;

    float Param::X_UP = 0;
    float Param::X_DOWN = 0;
    float Param::Y_UP = 0;
    float Param::Y_DOWN = 0;
    float Param::Z_UP = 0;
    float Param::Z_DOWN = 0;
    float Param::ANGLEX_UP = 0;
    float Param::ANGLEX_DOWN = 0;
    float Param::ANGLEY_UP = 0;
    float Param::ANGLEY_DOWN = 0;
    float Param::ANGLEZ_UP = 0;
    float Param::ANGLEZ_DOWN = 0;

    float Param::STEP_OFFSET = 0;
    float Param::STEP_ANGLE = 0;
    float Param::MAX_HIT_RATE = 0;
    int Param::MAX_ITER_NUMBER = 0;

    int Param::DEBUG_SHOW_IMAGE = 0;
    int Param::DEBUG_SHOW_POINTCLOUD = 0;
    int Param::DEBUG_SHOW_COORDINATE = 0;
    int Param::DEBUG_PROCESS = 0;
    int Param::DEBUG_CALCULATE = 0;

}