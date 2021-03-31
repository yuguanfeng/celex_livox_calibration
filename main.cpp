#include <iostream>
#include <string.h>
#include <fstream>
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
    fs["max_hit_rate"] >> calib::Param::MAX_HIT_RATE;
    fs["max_iter_number"] >> calib::Param::MAX_ITER_NUMBER;

    fs["debug_show_image"] >> calib::Param::DEBUG_SHOW_IMAGE;
    fs["debug_show_pointcloud"] >> calib::Param::DEBUG_SHOW_POINTCLOUD;
    fs["debug_show_coordinate"] >> calib::Param::DEBUG_SHOW_COORDINATE;
    fs["debug_process"] >> calib::Param::DEBUG_PROCESS;
    fs["debug_calculate"] >> calib::Param::DEBUG_CALCULATE;

}



int main(void){

    cout << "～～～～～～～～begin load param～～～～～～～～" << endl;
    //载入参数
    LoadParam();
    cout << "camera_matrix:" << endl << calib::Param::camera_matrix << endl;

    float tran_x = calib::Param::OFFSET_X;
    float tran_y = calib::Param::OFFSET_Y;
    float tran_z = calib::Param::OFFSET_Z;
    float yaw = calib::Param::ANGLE_Z / 180 * CV_PI;
    float pitch = calib::Param::ANGLE_Y / 180 * CV_PI;
    float roll = calib::Param::ANGLE_X / 180 * CV_PI;
    
    ExtriCal ex_cal;
    vector<cv::Point2f> roi_points;
    vector< vector<cv::Point2f> > roi_points_set;

//    PointCloud::Ptr pointcloud(new PointCloud);
//    PointCloud::Ptr roi_pointcloud(new PointCloud);
 
    ifstream txt_reader;
    vector<cv::Point3f> roi_3d_points;
    vector< vector<cv::Point3f> > roi_3d_points_set;
    

    //在外参迭代计算之前，先对图像进行处理，得到DATASET_NUMBERD个像素集；
    //对点云坐标进行读取（默认是已经用cloudcompare处理后的），得到DATASET_NUMBER个三维点集
    for (int i = 0; i < calib::Param::DATASET_NUMBER; i++){

        if(calib::Param::DEBUG_PROCESS == 1){
            cout << "～～～～～～～～No. " << i << "～～～～～～～～" << endl;
            cout << "～～～～～～～～begin process image～～～～～～～～" << endl;
        }

        //读取图像，并处理得到白板所在像素集合
        string img_path = "/home/yuguanfeng/celex_livox_calibration/data/camera/" 
                        + std::to_string(i) + ".bmp";    //注意要绝对路径，相对路径会报错
        cv::Mat img = cv::imread(img_path);
        if(img.empty()){
            cout << "Image loading failed !" << endl;
            return -1;
        }
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);   //测试时输入时rgb图像，先进行灰度转化，实际事件相机得到就是灰度图像
/*         if(ex_cal.processImage(img, roi_points, calib::Param::GRAY_THRESHOLD)){
            if(calib::Param::DEBUG_PROCESS == 1){
                cout << "The number of roi_points in No." << i << " image is " << roi_points.size() << endl;
            } 
        }else{
            cout << "Process image failed !" << endl;
                return -1;
        } */
        roi_points.push_back(cv::Point2f(2,2143)) ;
        roi_points.push_back(cv::Point2f(-53,2024));
        roi_points.push_back(cv::Point2f(-249,1398));
        roi_points.push_back(cv::Point2f(-261,1372));
        roi_points.push_back(cv::Point2f(-411,1140));
        roi_points.push_back(cv::Point2f(-419,1130));
        roi_points.push_back(cv::Point2f(-852,919));
        roi_points.push_back(cv::Point2f(-855,874));
        roi_points.push_back(cv::Point2f(-849,827));

        roi_points.push_back(cv::Point2f(-0,2195));
        roi_points.push_back(cv::Point2f(-58,2077));
        roi_points.push_back(cv::Point2f(-270,1426));
        roi_points.push_back(cv::Point2f(-432,1186));
        roi_points.push_back(cv::Point2f(-886,920));

        roi_points.push_back(cv::Point2f(-39,1982)) ;
        roi_points.push_back(cv::Point2f(-246,1331));
        roi_points.push_back(cv::Point2f(-389,1097));
        roi_points.push_back(cv::Point2f(-413,1062));
        roi_points.push_back(cv::Point2f(-822,838));
        roi_points.push_back(cv::Point2f(-823,796));

        roi_points_set.push_back(roi_points);

        //清空vector数据，否则会累计
        roi_points.clear();

/*         cout << "～～～～～～～～begin process pointcloud～～～～～～～～" << endl;
        //读取点云，并处理得到白板所在点云集合
    //  string ply_path = "/home/yuguanfeng/celex_livox_calibration/data/lidar/" 
    //                    + std::to_string(i) + ".ply";    //注意要绝对路径，相对路径会报错
        string ply_path = "/home/yuguanfeng/celex_livox_calibration/data/lidar/0.ply" ;
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(ply_path, *pointcloud) == -1) {       
            PCL_ERROR("PointCloud loading failed !\n");
            system("pause");
            return -1;
        }

        pcl::PLYReader reader;
        reader.read(ply_path, *pointcloud);
        cout << "test6......." << endl;
        cout << "pointcloud's width:" << pointcloud->width << ", height:" << pointcloud->height << endl; 

        pcl::visualization::CloudViewer viewer("PointCloud Viewer");
        viewer.showCloud(pointcloud);
        while (!viewer.wasStopped()){
            cout << "show pointcloud" << endl;
        } 

        if(ex_cal.processPointCloud(pointcloud, roi_pointcloud, calib::Param::POINTCLOUD_THRESHOLD)){
            cout << "No." << i << " pointcloud's width:" << pointcloud->width << ", height:" << pointcloud->height << endl;
            cout << "No." << i << " roi_pointcloud's width:" << roi_pointcloud->width << ", height:" << roi_pointcloud->height << endl;
        }else{
            cout << "Process PointCloud failed !" << endl;
            return -1;
        }  */
        if(calib::Param::DEBUG_PROCESS == 1){
            cout << "～～～～～～～～begin read txt～～～～～～～～" << endl;
        }
        string txt_path = "/home/yuguanfeng/celex_livox_calibration/data/lidar/" 
                        + std::to_string(i) + ".txt";    //注意要绝对路径，相对路径会报错
        txt_reader.open(txt_path);
        assert(txt_reader.is_open());

        cv::Point3f temp;
        float temp1 = 0, temp2 = 0, temp3 = 0;
        for(int j = 0; !txt_reader.eof(); j++){
            txt_reader >> temp1 >> temp2 >> temp3;
            temp.x = temp1;
            temp.y = temp2;
            temp.z = temp3;
            roi_3d_points.push_back(temp);
        }
        if(calib::Param::DEBUG_PROCESS == 1){
            cout << "The number of roi_3d_points in No." << i << " pointcloud is " << roi_3d_points.size() << endl;
            cout << roi_3d_points[0].x << " " << roi_3d_points[0].y << " " << roi_3d_points[0].z << endl << endl;
        }        
        roi_3d_points_set.push_back(roi_3d_points);

        //每一次都清空vector数据,关闭txt_reader
        txt_reader.close();
        roi_3d_points.clear();  

    }

    int iter_number = 0;
    float last_hit_rate = 0;
    int best_iter = 0;
    float best_hit_rate = 0;
    cv::Mat best_T_l2c = cv::Mat::zeros(cv::Size(4,4), CV_64FC1);

    while(1){

        iter_number++;    //计算迭代次数
        cout << endl << "～～～～～～～～Iteration " << iter_number << "～～～～～～～～" << endl;

        //P_camera = T_l2c * P_lidar; 相机坐标系为参考坐标系，雷达坐标系为动坐标系
        cv::Mat T_l2c = cv::Mat::zeros(cv::Size(4,4), CV_64FC1);
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

        int total_number_hit = 0;
        int total_number_3d_points = 0;
        float total_hit_rate = 0; 

        for (int i = 0; i < calib::Param::DATASET_NUMBER; i++){
            if(calib::Param::DEBUG_CALCULATE == 1){
                cout << "～～～～～～～～begin calculate No." << i << " hit-rate～～～～～～～～" << endl;
            }
            int number_hit = 0;
            int number_3d_points = roi_3d_points_set[i].size();
            float hit_rate = 0;
            
            for(int j = 0; j < number_3d_points; j++){
                //点云投影
                cv::Point3f temp_3d_point = roi_3d_points_set[i][j];
                cv::Point2f temp_pixel_point;
                ex_cal.project(temp_3d_point, temp_pixel_point, T_l2c);

                //判断是否中靶
                int hit_flag = 0;
                hit_flag = ex_cal.judgeHit(temp_pixel_point, roi_points_set[i]);
                if(hit_flag == 1){
                    number_hit++;
                }
            }
            hit_rate = (float)number_hit / (float)number_3d_points;//不转化为浮点型的话
                                                                   //则先整型除法再转化为浮点数，结果一般为0

            if(calib::Param::DEBUG_CALCULATE == 1){
                cout << "The number of hit in No." << i << " dataset is " << number_hit << endl;
                cout << "The number of roi_3d_points in No." << i << " dataset is " << number_3d_points << endl;
                cout << "The hit_rate of No. " << i << " dataset is " << hit_rate << endl;
            }

            total_number_hit = total_number_hit + number_hit;
            total_number_3d_points = total_number_3d_points + number_3d_points;
        }

        total_hit_rate = (float)total_number_hit / (float)total_number_3d_points;
        last_hit_rate = total_hit_rate;
        cout << "～～～～～～～～begin calculate total hit-rate～～～～～～～～" << endl;
        cout << "The total number of hit is " << total_number_hit << endl;
        cout << "The total number of roi_3d_points is " << total_number_3d_points << endl;
        cout << "The total hit_rate is " << total_hit_rate << endl;


        if(total_hit_rate > best_hit_rate || fabs(total_hit_rate - best_hit_rate) < 1e-6){//加上等于，那么不会出现
            best_iter = iter_number;                                                      //若total_hit_rate一直为零，则最后T也为零的情况
            best_hit_rate = total_hit_rate;                                               //即没有赋值
            for(int k = 0; k < 4; k++){
                for(int l = 0; l < 4; l++){
                    best_T_l2c.at<double>(k,l) = T_l2c.at<double>(k,l);
                }
            }
        }

        //达到一个差不多的上靶率即可退出
        if(best_hit_rate > calib::Param::MAX_HIT_RATE){
            cout << endl << "!!!Already got a high hit-rate!!!" << endl;
            break;
        }
        //即使未达到比较好的上靶率，达到一定迭代次数也退出
        if(iter_number > calib::Param::MAX_ITER_NUMBER){
            cout << endl << "!!!Already reached a max iter-number!!!" << endl;
            break;
        }

    }

    cout << "Best iter is " << best_iter << endl;
    cout << "Best hit-rate is " << best_hit_rate << endl;
    cout << "Best T_l2c is: " << endl << best_T_l2c << endl;

    return 0;

}