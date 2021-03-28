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

}



int main(void){

    //载入参数
    LoadParam();
    cout << calib::Param::camera_matrix << endl;

    cout << "test0......." << endl;
    vector<cv::Point2f> roi_points;
    cout << "test1......." << endl;
    PointCloud::Ptr pointcloud(new PointCloud);
    cout << "test2......." << endl;
    PointCloud::Ptr roi_pointcloud(new PointCloud);
    cout << "test3......." << endl;
    ExtriCal ex_cal;
    cout << "test4......." << endl;


    // for (int i = 0; i < DATASET_NUMBER; i++){

/*         cout << "～～～～～～～～begin process image～～～～～～～～" << endl;
        //读取图像，并处理得到白板所在像素集合
        string img_path = "/home/yuguanfeng/celex_livox_calibration/data/camera/" 
                         + std::to_string(i) + ".bmp";    //注意要绝对路径，相对路径会报错
        cv::Mat img = cv::imread(img_path);
        if(img.empty()){
            cout << "Image loading failed !" << endl;
            return -1;
        }
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);   //测试时输入时rgb图像，先进行灰度转化，实际事件相机得到就是灰度图像
        if(ex_cal.processImage(img, roi_points, GRAY_THRESHOLD)){
            cout << "The number of roi_points in No." << i << " image is " << roi_points.size() << endl << endl;
        }else{
            cout << "Process image failed !" << endl;
            return -1;
        } */

/*         cout << "～～～～～～～～begin process pointcloud～～～～～～～～" << endl;

        //读取点云，并处理得到白板所在点云集合
//        string ply_path = "/home/yuguanfeng/celex_livox_calibration/data/lidar/" 
//                         + std::to_string(i) + ".ply";    //注意要绝对路径，相对路径会报错
        string ply_path = "/home/yuguanfeng/celex_livox_calibration/data/lidar/0.ply" ;
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(ply_path, *pointcloud) == -1) {       
            PCL_ERROR("PointCloud loading failed !\n");
            system("pause");
            return -1;
        }

        if(ex_cal.processPointCloud(pointcloud, roi_pointcloud, POINTCLOUD_THRESHOLD)){
            cout << "No." << i << " pointcloud's width:" << pointcloud->width << ", height:" << pointcloud->height << endl;
            cout << "No." << i << " roi_pointcloud's width:" << roi_pointcloud->width << ", height:" << roi_pointcloud->height << endl;
        }else{
            cout << "Process PointCloud failed !" << endl;
            return -1;
        }

    } */

    //    string ply_path = "/home/yuguanfeng/celex_livox_calibration/data/lidar/1.ply" ;
/* 
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(ply_path, *pointcloud) == -1) {       
            PCL_ERROR("PointCloud loading failed !\n");
            system("pause");
            return -1;
        } */

/*         pcl::PLYReader reader;
        reader.read(ply_path, *pointcloud);
        cout << "test6......." << endl;
        cout << "pointcloud's width:" << pointcloud->width << ", height:" << pointcloud->height << endl; */

/*         pcl::visualization::CloudViewer viewer("PointCloud Viewer");
        viewer.showCloud(pointcloud);
        while (!viewer.wasStopped()){
            cout << "show pointcloud" << endl;
        } */
    
        cout << "～～～～～～～～begin calculate T ～～～～～～～～" << endl;



    //}
    return 0;

}