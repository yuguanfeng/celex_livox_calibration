#include <iostream>
#include <string.h>
#include "common_PCL.h"
#include "Param.h"
#include "ExtriCal.h"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

void LoadParam(){
    
    cv::FileStorage fs;
    fs.open("/home/yuguanfeng/celex_livox_calibration/param/config.yml",cv::FileStorage::READ);
    assert(fs.isOpened());

    fs["dataset_number"] >> calib::Param::DATASET_NUMBER;
    fs["gray_threshold"] >> calib::Param::GRAY_THRESHOLD;
    fs["pointcloud_threshold"] >> calib::Param::POINTCLOUD_THRESHOLD;

    fs["width"] >> calib::Param::WIDTH;
    fs["height"] >> calib::Param::HEIGHT;

    fs["camera_matrix"] >> calib::Param::camera_matrix;
    fs["distortion_coefficients"] >> calib::Param::distort;
    cv::initUndistortRectifyMap(calib::Param::camera_matrix, calib::Param::distort, cv::noArray(), cv::noArray(),
                                cv::Size(calib::Param::WIDTH, calib::Param::HEIGHT), CV_32FC1, calib::Param::MAPX, calib::Param::MAPY);

    fs["width"] >> calib::Param::WIDTH;
    fs["height"] >> calib::Param::HEIGHT;

    fs["offset_x"] >> calib::Param::OFFSET_X;
    fs["offset_y"] >> calib::Param::OFFSET_Y;
    fs["offset_z"] >> calib::Param::OFFSET_Z;
    fs["angle_x"] >> calib::Param::ANGLE_X;
    fs["angle_y"] >> calib::Param::ANGLE_Y;
    fs["angle_z"] >> calib::Param::ANGLE_Z;

    fs["step_offset"] >> calib::Param::STEP_OFFSET;
    fs["step_angle"] >> calib::Param::STEP_ANGLE;

}



int main(void){

    //载入参数
    LoadParam();
    cout << calib::Param::camera_matrix << endl;

    //P_camera = T_l2c * P_lidar; 相机坐标系为参考坐标系，雷达坐标系为动坐标系
    cv::Mat T_l2c = cv::Mat::zeros(cv::Size(4,4), CV_64FC1);
    float tran_x = calib::Param::OFFSET_X;
    float tran_y = calib::Param::OFFSET_Y;
    float tran_z = calib::Param::OFFSET_Z;
    float yaw = calib::Param::ANGLE_Z / 180 * CV_PI;
    float pitch = calib::Param::ANGLE_Y / 180 * CV_PI;
    float roll = calib::Param::ANGLE_X / 180 * CV_PI;

    vector<cv::Point2f> roi_points;
    PointCloud::Ptr pointcloud(new PointCloud);
    PointCloud::Ptr roi_pointcloud(new PointCloud);
    ExtriCal ex_cal;

    //while(1){

        //在每一次迭代开始，改变z轴平移量和绕z轴旋转量
        tran_x = tran_x;
        tran_y = tran_y;
        tran_z = tran_z + calib::Param::STEP_OFFSET;
        yaw = yaw + calib::Param::STEP_ANGLE / 180 * CV_PI;
        pitch = pitch;
        roll = roll;

        cout << "～～～～～～～～begin calculate T_Lidar2Camera～～～～～～～～" << endl;
        if(ex_cal.calculateT(T_l2c, tran_x, tran_y, tran_z, yaw, pitch, roll)){
            cout << "The transform matrix from lidar to camera is: " << endl << T_l2c << endl;
        }else{
            cout << "Calculate T failed !" << endl;
            return -1;
        }       



    
        //for (int i = 0; i < calib::Param::DATASET_NUMBER; i++){

/*             cout << "～～～～～～～～begin process image～～～～～～～～" << endl;
            //读取图像，并处理得到白板所在像素集合
            string img_path = "/home/yuguanfeng/celex_livox_calibration/data/camera/" 
                            + std::to_string(i) + ".bmp";    //注意要绝对路径，相对路径会报错
            cv::Mat img = cv::imread(img_path);
            if(img.empty()){
                cout << "Image loading failed !" << endl;
                return -1;
            }
            cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);   //测试时输入时rgb图像，先进行灰度转化，实际事件相机得到就是灰度图像
            if(ex_cal.processImage(img, roi_points, calib::Param::GRAY_THRESHOLD)){
                cout << "The number of roi_points in No." << i << " image is " << roi_points.size() << endl << endl;
            }else{
                cout << "Process image failed !" << endl;
                return -1;
            } */

/*             cout << "～～～～～～～～begin process pointcloud～～～～～～～～" << endl;
            //读取点云，并处理得到白板所在点云集合
    //        string ply_path = "/home/yuguanfeng/celex_livox_calibration/data/lidar/" 
    //                         + std::to_string(i) + ".ply";    //注意要绝对路径，相对路径会报错
            string ply_path = "/home/yuguanfeng/celex_livox_calibration/data/lidar/0.ply" ;
            if (pcl::io::loadPLYFile<pcl::PointXYZ>(ply_path, *pointcloud) == -1) {       
                PCL_ERROR("PointCloud loading failed !\n");
                system("pause");
                return -1;
            }

            if(ex_cal.processPointCloud(pointcloud, roi_pointcloud, calib::Param::POINTCLOUD_THRESHOLD)){
                cout << "No." << i << " pointcloud's width:" << pointcloud->width << ", height:" << pointcloud->height << endl;
                cout << "No." << i << " roi_pointcloud's width:" << roi_pointcloud->width << ", height:" << roi_pointcloud->height << endl;
            }else{
                cout << "Process PointCloud failed !" << endl;
                return -1;
            } */

            //string ply_path = "/home/yuguanfeng/celex_livox_calibration/data/lidar/1.ply" ;

/*             if (pcl::io::loadPLYFile<pcl::PointXYZ>(ply_path, *pointcloud) == -1) {       
                PCL_ERROR("PointCloud loading failed !\n");
                system("pause");
                return -1;
            } */

/*             pcl::PLYReader reader;
            reader.read(ply_path, *pointcloud);
            cout << "test6......." << endl;
            cout << "pointcloud's width:" << pointcloud->width << ", height:" << pointcloud->height << endl; */

/*             pcl::visualization::CloudViewer viewer("PointCloud Viewer");
            viewer.showCloud(pointcloud);
            while (!viewer.wasStopped()){
                cout << "show pointcloud" << endl;
            } */
    
        //}


    //}

    return 0;

}