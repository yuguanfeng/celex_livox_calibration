#include <iostream>
#include <string.h>
#include <fstream>
#include "Param.h"
#include "ExtriCal.h"

using namespace std;

void LoadParam(){
    
    cv::FileStorage fs;
    fs.open("/home/yuguanfeng/celex_livox_calibration/param/config.yml",cv::FileStorage::READ);
    assert(fs.isOpened());

    fs["width"] >> calib::Param::WIDTH;
    fs["height"] >> calib::Param::HEIGHT;

    fs["camera_matrix"] >> calib::Param::camera_matrix;
    fs["distortion_coefficients"] >> calib::Param::distort;
//   cv::initUndistortRectifyMap(calib::Param::camera_matrix, calib::Param::distort, cv::noArray(), cv::noArray(),
//                                cv::Size(calib::Param::WIDTH, calib::Param::HEIGHT), CV_32FC1, calib::Param::MAPX, calib::Param::MAPY);

    fs["offset_x"] >> calib::Param::OFFSET_X;
    fs["offset_y"] >> calib::Param::OFFSET_Y;
    fs["offset_z"] >> calib::Param::OFFSET_Z;
    fs["angle_x"] >> calib::Param::ANGLE_X;
    fs["angle_y"] >> calib::Param::ANGLE_Y;
    fs["angle_z"] >> calib::Param::ANGLE_Z;

    fs["y_up"] >> calib::Param::Y_UP;
    fs["y_down"] >> calib::Param::Y_DOWN;
    fs["z_up"] >> calib::Param::Z_UP;
    fs["z_down"] >> calib::Param::Z_DOWN;
    fs["angleY_up"] >> calib::Param::ANGLEY_UP;
    fs["angleY_down"] >> calib::Param::ANGLEY_DOWN;

    fs["dataset_number"] >> calib::Param::DATASET_NUMBER;
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
    /***载入参数***/
    LoadParam();
    cout << "camera_matrix:" << endl << calib::Param::camera_matrix << endl;

    float tran_x = calib::Param::OFFSET_X;
    float tran_y = calib::Param::OFFSET_Y;
    float tran_z = calib::Param::OFFSET_Z;
    float pitch = calib::Param::ANGLE_X / 180 * CV_PI;
    float yaw = calib::Param::ANGLE_Y / 180 * CV_PI;
    float roll = calib::Param::ANGLE_Z / 180 * CV_PI;
    
    ExtriCal ex_cal;
    vector<cv::Point2f> roi_points;
    vector< vector<cv::Point2f> > roi_points_set;
    ifstream txt_reader;
    vector<cv::Point3f> roi_3d_points;
    vector< vector<cv::Point3f> > roi_3d_points_set;
    
    /***读取手动选择的各图四个角点坐标***/
    if(calib::Param::DEBUG_PROCESS == 1){
        cout << "～～～～～～～～begin read corners txt～～～～～～～～" << endl;
    }
    ifstream corner_txt_reader;
    vector<cv::Point2f> corner_points;
    vector< vector<cv::Point2f> > corner_points_set;
    string corner_txt_path = "/home/yuguanfeng/celex_livox_calibration/data/camera/corners.txt";//注意要绝对路径，相对路径会报错
//    string corner_txt_path = "/home/yuguanfeng/celex_livox_calibration/data/camera/corners_little.txt";
    corner_txt_reader.open(corner_txt_path);
    assert(corner_txt_reader.is_open());

    cv::Point2f corner1, corner2, corner3, corner4;
    float temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0;
    float temp5 = 0, temp6 = 0, temp7 = 0, temp8 = 0;//四个角点，八个坐标
    for(int k = 0; !corner_txt_reader.eof(); k++){
        corner_txt_reader >> temp1 >> temp2 >> temp3 >> temp4 >> temp5 >> temp6 >> temp7 >> temp8;
        corner1.x = temp1;
        corner1.y = temp2;
        corner_points.push_back(corner1);
        corner2.x = temp3;
        corner2.y = temp4;
        corner_points.push_back(corner2);
        corner3.x = temp5;
        corner3.y = temp6;
        corner_points.push_back(corner3);
        corner4.x = temp7;
        corner4.y = temp8;
        corner_points.push_back(corner4);
        corner_points_set.push_back(corner_points);
        corner_points.clear();
    }
    corner_txt_reader.close();
    if(calib::Param::DEBUG_PROCESS == 1){
        cout << "The number of corner_points_set is " << corner_points_set.size() << endl;
        cout << "corner_points_set[3].corner1: " << corner_points_set[3][0] << endl;
        cout << "corner_points_set[3].corner2: " << corner_points_set[3][1] << endl;
        cout << "corner_points_set[3].corner3: " << corner_points_set[3][2] << endl;
        cout << "corner_points_set[3].corner4: " << corner_points_set[3][3] << endl << endl;
    }    

    /***在外参迭代计算之前，先对图像和点云进行处理，得到DATASET_NUMBERD个数据集***/
    for (int i = 0; i < calib::Param::DATASET_NUMBER; i++){

        /***读取图像，并处理得到白板所在像素集合***/
        if(calib::Param::DEBUG_PROCESS == 1){
            cout << "～～～～～～～～No. " << i << "～～～～～～～～" << endl;
            cout << "～～～～～～～～begin process image～～～～～～～～" << endl;
        }
        string img_path = "/home/yuguanfeng/celex_livox_calibration/data/camera/" 
                        + std::to_string(i) + ".JPG";    
//        string img_path = "/home/yuguanfeng/celex_livox_calibration/data/camera/7.JPG";
        cv::Mat img = cv::imread(img_path);
        if(img.empty()){
            cout << "Image loading failed !" << endl;
            return -1;
        }
        if(ex_cal.processImage(img, corner_points_set[i],roi_points)){
            if(calib::Param::DEBUG_PROCESS == 1){
                cout << "The number of roi_points in No." << i << " image is " << roi_points.size() << endl;
            } 
        }else{
            cout << "Process image failed !" << endl;
                return -1;
        }
        roi_points_set.push_back(roi_points);
        roi_points.clear();//清空vector数据，否则会累计
        
        /***对点云坐标进行读取（默认是已经用cloudcompare处理后的），得到DATASET_NUMBER个三维点集***/
        if(calib::Param::DEBUG_PROCESS == 1){
            cout << "～～～～～～～～begin read lidar txt～～～～～～～～" << endl;
        }
        string txt_path = "/home/yuguanfeng/celex_livox_calibration/data/lidar/" 
                        + std::to_string(i) + ".txt";    //注意要绝对路径，相对路径会报错
//        string txt_path = "/home/yuguanfeng/celex_livox_calibration/data/lidar/0" 
//                        + std::to_string(i) + ".txt";
        txt_reader.open(txt_path);
        assert(txt_reader.is_open());

        cv::Point3f temp;
        float temp1 = 0, temp2 = 0, temp3 = 0;
        float temp4 = 0, temp5 = 0, temp6 = 0, temp7 = 0;//rgb+indenity 这里只读没保存
        for(int j = 0; !txt_reader.eof(); j++){
            txt_reader >> temp1 >> temp2 >> temp3 >> temp4 >> temp5 >> temp6 >> temp7;
            temp.x = temp1;
            temp.y = temp2;
            temp.z = temp3;
            roi_3d_points.push_back(temp);
        }
        if(calib::Param::DEBUG_PROCESS == 1){
            cout << "The number of roi_3d_points in No." << i << " pointcloud is " << roi_3d_points.size() << endl;
            cout << "roi_3d_points[0].x: " << roi_3d_points[0].x 
             << "    roi_3d_points[0].y: " << roi_3d_points[0].y
             << "    roi_3d_points[0].z: " << roi_3d_points[0].z << endl << endl;
        }        
        roi_3d_points_set.push_back(roi_3d_points);

        txt_reader.close();//每一次都关闭txt_reader，清空vector数据
        roi_3d_points.clear();
    }
    
    srand(time(0));//以系统时间为随机数种子，避免每次运行产生随机数列相同
    int iter_number = 0;
    float last_hit_rate = 0;
    int best_iter = 0;
    float best_hit_rate = 0;
    float best_tran_x = 0, best_tran_y = 0, best_tran_z = 0;
    float best_yaw = 0, best_pitch = 0, best_roll = 0;
    cv::Mat best_T_l2c = cv::Mat::zeros(cv::Size(4,4), CV_64FC1);

    while(1){

        iter_number++;//计算迭代次数
        cout << endl << "～～～～～～～～Iteration " << iter_number << "～～～～～～～～" << endl;

        //在每一次迭代开始，改变yz轴的平移量和绕y旋转量【随机选择】
        int step_flag = (rand()%(6-0))+0;
        switch (step_flag)
        {
        case 0:
            tran_y = tran_y + calib::Param::STEP_OFFSET;
            break;
        case 1:
            tran_y = tran_y - calib::Param::STEP_OFFSET;
            break;
        case 2:
            tran_z = tran_z + calib::Param::STEP_OFFSET;
            break;
        case 3:
            tran_z = tran_z - calib::Param::STEP_OFFSET;
            break;
        case 4:
            yaw = yaw + calib::Param::STEP_ANGLE / 180 * CV_PI;
            break;            
        case 5:
            yaw = yaw - calib::Param::STEP_ANGLE / 180 * CV_PI;
            break;
        default:
            cout << "step_flag error!" << endl;
            return -1;
        }
        //判断是否到达上下限，如到达则不再增加或减少
        if(tran_y > calib::Param::Y_UP){
            tran_y = calib::Param::Y_UP;
        }
        if(tran_y < calib::Param::Y_DOWN){
            tran_y = calib::Param::Y_DOWN;
        }
        if(tran_z > calib::Param::Z_UP){
            tran_z = calib::Param::Z_UP;
        }
        if(tran_z < calib::Param::Z_DOWN){
            tran_z = calib::Param::Z_DOWN;
        }
        if(yaw > (calib::Param::ANGLEY_UP/180*CV_PI)){
            yaw = calib::Param::ANGLEY_UP/180*CV_PI;
        }
        if(yaw < (calib::Param::ANGLEY_DOWN/180*CV_PI)){
            yaw = calib::Param::ANGLEY_DOWN/180*CV_PI;
        }
        cout << "tran_x:" << tran_x << " tran_y:" << tran_y << " tran_z:" << tran_z << endl;
        cout << "pitch:" << pitch*180/CV_PI << " yaw:" << yaw*180/CV_PI << " roll:" << roll*180/CV_PI << endl;

        //计算当前外参
        cv::Mat T_l2c = cv::Mat::zeros(cv::Size(4,4), CV_64FC1);
        cout << "～～～～～～～～begin calculate T_Lidar2Camera～～～～～～～～" << endl;
        if(ex_cal.calculateT(T_l2c, tran_x, tran_y, tran_z, roll, yaw, pitch)){
            cout << "The transform matrix from lidar to camera is: " << endl << T_l2c << endl;
        }else{
            cout << "Calculate T failed !" << endl;
            return -1;
        } 

        int total_number_hit = 0;
        int total_number_3d_points = 0;
        float total_hit_rate = 0; 

        //计算整体上靶率
        for (int i = 0; i < calib::Param::DATASET_NUMBER; i++){

            if(calib::Param::DEBUG_CALCULATE == 1){
                cout << "～～～～～～～～begin calculate No." << i << " hit-rate～～～～～～～～" << endl;
            }
            int number_hit = 0;
            int number_3d_points = roi_3d_points_set[i].size();
            float hit_rate = 0;

            for(int j = 0; j < number_3d_points; j++){
                cv::Point3f temp_3d_point = roi_3d_points_set[i][j];
                cv::Point2f temp_pixel_point;   
                ex_cal.projectL2C(temp_3d_point, temp_pixel_point, T_l2c);//点云投影
                temp_pixel_point.x = temp_pixel_point.x*((float)1280/(float)8256);//resize,记得转浮点型！！！
                temp_pixel_point.y = temp_pixel_point.y*((float)800/(float)5504);//否则为零！！！
                int hit_flag = 0;
                hit_flag = ex_cal.judgeHit(temp_pixel_point, roi_points_set[i]);//判断是否中靶
                if(hit_flag == 1){
                    number_hit++;
                }
            }
            hit_rate = (float)number_hit / (float)number_3d_points;//不转化为浮点型的话,则先整型除法再转化为浮点数，结果一般为0
                                                                                
            if(calib::Param::DEBUG_CALCULATE == 1){
                cout << "The number of hit in No." << i << " dataset is " << number_hit << endl;
                cout << "The number of roi_3d_points in No." << i << " dataset is " << number_3d_points << endl;
                cout << "The hit_rate of No." << i << " dataset is " << hit_rate << endl;
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
            best_tran_x = tran_x;
            best_tran_y = tran_y;
            best_tran_z = tran_z;
            best_yaw = yaw*180/CV_PI;
            best_pitch = pitch*180/CV_PI;
            best_roll = roll*180/CV_PI;                                           
            for(int k = 0; k < 4; k++){
                for(int l = 0; l < 4; l++){
                    best_T_l2c.at<double>(k,l) = T_l2c.at<double>(k,l);
                }
            }
        }

        if(best_hit_rate > calib::Param::MAX_HIT_RATE){//达到一个差不多的上靶率即可退出
            cout << endl << "!!!Already got a high hit-rate!!!" << endl;
            break;
        }
        if(iter_number > calib::Param::MAX_ITER_NUMBER){//即使未达到比较好的上靶率，达到一定迭代次数也退出
            cout << endl << "!!!Already reached a max iter-number!!!" << endl;
            break;
        }

    }

    cout << "Best iter is " << best_iter << endl;
    cout << "Best hit-rate is " << best_hit_rate << endl;
    cout << "Best tranx is " << best_tran_x << " trany is " << best_tran_y << " tranz is " << best_tran_z << endl;
    cout << "Best pitch is " << best_pitch << " yaw is " << best_yaw << " roll is " << best_roll << endl;
    cout << "Best T_l2c is " << endl << best_T_l2c << endl;

    /***测试效果图***/
    vector<cv::Point3f> test_3d_points;  
    ifstream test_reader;
    string test_path = "/home/yuguanfeng/celex_livox_calibration/data/lidar/d00.txt";
    test_reader.open(test_path);
    assert(test_reader.is_open());

    cv::Point3f test_temp;
    float test_temp1 = 0, test_temp2 = 0, test_temp3 = 0;
    float test_temp4 = 0, test_temp5 = 0, test_temp6 = 0, test_temp7, test_temp8 = 0;//rgb+indenity 这里只读没保存
    for(int j = 0; !test_reader.eof(); j++){
        test_reader >> test_temp1 >> test_temp2 >> test_temp3 
        >> test_temp4 >> test_temp5 >> test_temp6 >> test_temp7 >> test_temp8;
        test_temp.x = test_temp1;
        test_temp.y = test_temp2;
        test_temp.z = test_temp3;
        test_3d_points.push_back(test_temp);
    }
    test_reader.close();

    string img_test_path = "/home/yuguanfeng/celex_livox_calibration/data/camera/d0.JPG";
    cv::Mat img_test = cv::imread(img_test_path);
    if(img_test.empty()){
        cout << "Image loading failed !" << endl;
        return -1;
    }
    cv::resize(img_test, img_test, cv::Size(1280, 800));

    for(int k = 0; k < test_3d_points.size(); k++){
        cv::Point3f test_3d_point = test_3d_points[k];
        cv::Point2f test_pixel_point;   
        ex_cal.projectL2C(test_3d_point, test_pixel_point, best_T_l2c);//点云投影
        test_pixel_point.x = test_pixel_point.x*((float)1280/(float)8256);//resize,记得转浮点型！！！
        test_pixel_point.y = test_pixel_point.y*((float)800/(float)5504);//否则为零！！！
        img_test.at<cv::Vec3b>(test_pixel_point.y,test_pixel_point.x)[0] = 0;
        img_test.at<cv::Vec3b>(test_pixel_point.y,test_pixel_point.x)[1] = 0;
        img_test.at<cv::Vec3b>(test_pixel_point.y,test_pixel_point.x)[2] = 255;
    }
    cv::namedWindow("img_test",cv::WINDOW_NORMAL);
    cv::imshow("img_test",img_test);
    cv::waitKey(0);

    return 0;

}