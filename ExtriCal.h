#include <opencv2/opencv.hpp>
#include "common_PCL.h"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ExtriCal{

public:
    ExtriCal();
    ~ExtriCal(){}
    
    bool processImage(cv::Mat& _frame, vector<cv::Point2f>& _roi_points, int _gray_threshold);
    bool processPointCloud(PointCloud::Ptr _point_cloud, PointCloud::Ptr _roi_point_cloud, float _pointcloud_threshold);
    
};