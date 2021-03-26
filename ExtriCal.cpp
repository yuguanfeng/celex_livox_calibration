#include "ExtriCal.h"

#define DEBUG_SHOW_IMAGE
#define DEBUG_SHOW_POINTCLOUD

using namespace std;
using namespace cv;

/**
 * @brief 解析函数
 */
ExtriCal::ExtriCal(){}

/**
 * @brief 处理图像，选出一定阈值内的点集，作为靶子
 */
ExtriCal::processImage(Mat& _frame, vector<Point2f>& _roi_points, int _threshold){

    img = _frame.clone;

#ifdef DEBUG_SHOW_IMAGE
    namedWindow("img_src",WINDOW_NORMAL);
    imshow("img_src",img)
#endif

    int row = img.rows;
    int col = img.cols;
    //用指针访问像素
    unchar *p;
    for(int i = 0; i < row; i++){
        p = img.ptr<uchar>(i);          //获得每行首地址
        for(int j = 0; j < col; j++){
            int gray = p[j];
            if(gray > _threshold){      //大于阈值即为白板上的点 
                _roi_points.pushback(Point2f(i,j));

#ifdef DEBUG_SHOW_IMAGE
    circle(img, Point2f(i, j), 2, Scalar(0, 255, 0), 2);
    namedWindow("img_roi",WINDOW_NORMAL);
    imshow("img_roi",img)
    waitkey(0);
#endif

            }
        }
    }
    
}