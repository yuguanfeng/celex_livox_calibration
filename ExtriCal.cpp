#include "ExtriCal.h"

#define DEBUG_SHOW_IMAGE
#define DEBUG_SHOW_POINTCLOUD

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/**
 * @brief 解析函数
 */
ExtriCal::ExtriCal(){}

/**
 * @brief 处理图像，选出一定阈值内的点集，作为靶子
 */
bool ExtriCal::processImage(cv::Mat& _frame, vector<cv::Point2f>& _roi_points, int _gray_threshold){

    cv::Mat img = _frame.clone();

#ifdef DEBUG_SHOW_IMAGE
    cv::namedWindow("img_src",cv::WINDOW_NORMAL);
    cv::imshow("img_src",img);
#endif

//    int row = img.rows;
//    int col = img.cols;
    int row = 50;
    int col = 50;

    //用指针访问像素
    uchar *p;
    for(int i = 0; i < row; i++){
        p = img.ptr<uchar>(i);          //获得每行首地址
        for(int j = 0; j < col; j++){
            int gray = p[j];
            if(gray > _gray_threshold){      //大于阈值即为板上的点 
                cv::Point2f roi_point = cv::Point2f((float)i,(float)j);
                _roi_points.push_back(roi_point);
                p[j] = 0;
            }else{
                p[j] = 255;
            }
        }
    }
#ifdef DEBUG_SHOW_IMAGE
    cv::namedWindow("img_roi",cv::WINDOW_NORMAL);
    cv::imshow("img_roi",img);
    cv::waitKey(0);
#endif
    return true;
}

/**
 * @brief 处理点云，选出一定阈值内的点集，作为子弹
 */
bool ExtriCal::processPointCloud(PointCloud::Ptr _point_cloud, PointCloud::Ptr _roi_point_cloud, float _pointcloud_threshold){


#ifdef DEBUG_SHOW_POINTCLOUD
/*     pcl::visualization::CloudViewer viewer("PointCloud Viewer");
    viewer.showCloud(_point_cloud); */
#endif


#ifdef DEBUG_SHOW_POINTCLOUD

#endif

    _roi_point_cloud = _point_cloud;

    return true;
    
}

/**
 * @brief 计算外参函数
 */
bool ExtriCal::calculateT(cv::Mat& _T, float _tran_x, float _tran_y, float _tran_z, float _yaw, float _pitch, float _roll){

    float r1 = cos(_yaw)*cos(_pitch);
    float r2 = cos(_yaw)*sin(_pitch)*sin(_roll) - sin(_yaw)*cos(_roll);
    float r3 = cos(_yaw)*sin(_pitch)*cos(_roll) + sin(_yaw)*sin(_roll);
    float r4 = sin(_yaw)*cos(_pitch);
    float r5 = sin(_yaw)*sin(_pitch)*sin(_roll) + cos(_yaw)*cos(_roll);
    float r6 = sin(_yaw)*sin(_pitch)*cos(_roll) - cos(_yaw)*sin(_roll);
    float r7 = -sin(_pitch);
    float r8 = cos(_pitch)*sin(_roll);
    float r9 = cos(_pitch)*cos(_roll);

    _T = (cv::Mat_<double>(4, 4) << r1, r2, r3, _tran_x, r4, r5, r6, _tran_y, r7, r8, r9, _tran_z, 0, 0, 0, 1);

    return true;
}