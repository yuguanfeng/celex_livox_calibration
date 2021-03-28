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