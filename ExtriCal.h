#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class ExtriCal{

public:
    ExtriCal();
    ~ExtriCal(){}
    bool ProcessImage(Mat& _frame, vector<Point2f>& _roi_points, int _threshold);
    



    vector<Point2f> roi_points;


    
    
}