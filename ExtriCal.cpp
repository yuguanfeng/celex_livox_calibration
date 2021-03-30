#include "ExtriCal.h"
#include "Param.h"

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

/**
 * @brief 将点云投影到像素
 */
bool ExtriCal::project(cv::Point3f _3d_point, cv::Point2f& _pixel_point, cv::Mat _T){
    //将3x1的点云，转换为4x1的世界坐标，因为外参是4x4
    cv::Mat world_point = cv::Mat(4,1,CV_64FC1);
    world_point.at<double>(0,0) = _3d_point.x;
    world_point.at<double>(1,0) = _3d_point.y;
    world_point.at<double>(2,0) = _3d_point.z;
    world_point.at<double>(3,0) = 1;
    cout << "--------point in world--------" << endl;
    cout << world_point << endl;

    //由外参得到4x1的相机坐标系下坐标
    cv::Mat camera_point = cv::Mat(4,1,CV_64FC1);
    camera_point = _T * world_point;
    cout << "--------point in camera--------" << endl;
    cout << camera_point << endl;

    //由3x3内参矩阵转换为3x4的变换矩阵，因为相机坐标为4x1
    cv::Mat matrix_3x4_c2p = cv::Mat(3,4,CV_64FC1);
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            matrix_3x4_c2p.at<double>(i,j) = calib::Param::camera_matrix.at<double>(i,j);
        }
    }
    for(int i = 0; i < 3; i++){
        matrix_3x4_c2p.at<double>(i,3) = 0;
    }
    cout << "----------matrix_3x4_c2p--------" << endl;
    cout << matrix_3x4_c2p << endl;

    //由3x4的变换矩阵得到3x1的像素平面内的齐次坐标
    cv::Mat pixel_point = cv::Mat(3,1,CV_64FC1);
    pixel_point = matrix_3x4_c2p * camera_point / camera_point.at<double>(2,0);//除Z(深度)是得到归一化平面的坐标
    cout << "--------point in image(3x1)--------" << endl;
    cout << pixel_point << endl;
    
    //赋值到Point2f的像素坐标
    _pixel_point = cv::Point2f(pixel_point.at<double>(0,0),pixel_point.at<double>(1,0));
    cout << "--------point in image--------" << endl;
    cout << _pixel_point << endl;

    return true;

}