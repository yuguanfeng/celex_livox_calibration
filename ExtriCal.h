#include <opencv2/opencv.hpp>

using namespace std;

class ExtriCal{

public:
    ExtriCal();
    ~ExtriCal(){}
    
    bool processImage(cv::Mat& _frame, vector<cv::Point2f>& _roi_points, int _threshold);
    



    vector<cv::Point2f> roi_points;


    
    
};