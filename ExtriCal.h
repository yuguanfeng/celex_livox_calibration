#include <opencv2/opencv.hpp>

using namespace std;

class ExtriCal{

public:
    ExtriCal();
    ~ExtriCal(){}
    
    bool processImage(cv::Mat _frame, vector<cv::Point2f> _corner_points, vector<cv::Point2f>& _roi_points);
    bool calculateT(cv::Mat& _T, float _tran_x, float _tran_y, float _tran_z, float _roll, float _yaw, float _pitch);
    bool projectL2C(cv::Point3f _3d_point, cv::Point2f& _pixel_point, cv::Mat _T);
    int judgeHit(cv::Point2f _pixel_point, vector<cv::Point2f> _roi_points);
    
};