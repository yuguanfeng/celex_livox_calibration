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

    float Param::OFFSET_X = 0;
    float Param::OFFSET_Y = 0;
    float Param::OFFSET_Z = 0;
    float Param::ANGLE_X = 0;
    float Param::ANGLE_Y = 0;
    float Param::ANGLE_Z = 0;

}