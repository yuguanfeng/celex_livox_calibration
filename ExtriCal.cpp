#include "Param.h"
#include "ExtriCal.h"

using namespace std;

/**
 * @brief 解析函数
 */
ExtriCal::ExtriCal(){}

/**
 * @brief 处理图像，选出四边形内的点集，作为靶子
 */
bool ExtriCal::processImage(cv::Mat _frame, vector<cv::Point2f> _corner_points, vector<cv::Point2f>& _roi_points){

    cv::Mat img_src = _frame.clone();
    cv::cvtColor(img_src, img_src, cv::COLOR_BGR2GRAY);
    if(calib::Param::DEBUG_SHOW_IMAGE == 1){
        cv::namedWindow("img_src",cv::WINDOW_NORMAL);
        cv::imshow("img_src",img_src);
    } 

    cv::Mat img_resize;
    cv::resize(img_src, img_resize, cv::Size(1280, 800));
    if(calib::Param::DEBUG_SHOW_IMAGE == 1){
        cv::namedWindow("img_resize",cv::WINDOW_NORMAL);
        cv::imshow("img_resize",img_resize);
    } 

    //这里一定要注意顺时针是按xy坐标来的，而不是图像上看着的顺时针！！！
    cv::Point2f A = _corner_points[3];//图片左下角点
    cv::Point2f B = _corner_points[2];//图片右下角点
    cv::Point2f C = _corner_points[1];//图片右上角点
    cv::Point2f D = _corner_points[0];//图片左上角点 

    if(calib::Param::DEBUG_SHOW_IMAGE == 1){
        cv::circle(img_resize, A, 3, cv::Scalar(255,255,255),2);
        cv::circle(img_resize, B, 3, cv::Scalar(255,255,255),2);
        cv::circle(img_resize, C, 3, cv::Scalar(255,255,255),2);
        cv::circle(img_resize, D, 3, cv::Scalar(255,255,255),2);
        cv::namedWindow("img_corners",cv::WINDOW_NORMAL);
        cv::imshow("img_corners",img_resize);
    }
    
    int row = img_resize.rows;
    int col = img_resize.cols;
    float cross1,cross2,cross3,cross4;
    for(int i = 0; i < row; i++){
        for(int j = 0; j < col; j++){
            cv::Point2f M = cv::Point2f((float)j,(float)i); //计算坐标将i,j倒换
            cross1 = (B.x-A.x)*(M.y-A.y)-(M.x-A.x)*(B.y-A.y);
            cross2 = (C.x-B.x)*(M.y-B.y)-(M.x-B.x)*(C.y-B.y);
            cross3 = (D.x-C.x)*(M.y-C.y)-(M.x-C.x)*(D.y-C.y);
            cross4 = (A.x-D.x)*(M.y-D.y)-(M.x-D.x)*(A.y-D.y);
            //cout << cross1 << " " << cross2 << " " <<  cross3 << " " <<  cross4 << endl; 
            if(cross1<0 && cross2<0 && cross3<0 && cross4<0){
                img_resize.at<uchar>(i,j) = 0;
                _roi_points.push_back(M);
            //    cout << "IN" << endl;
            }else{
                img_resize.at<uchar>(i,j) = 255;
            }
        }
    }

    if(calib::Param::DEBUG_SHOW_IMAGE == 1){
        cv::namedWindow("img_roi",cv::WINDOW_NORMAL);
	    cv::imshow("img_roi",img_resize);
        cv::waitKey(0);
    }

    return true;
}

/**
 * @brief 计算外参函数,采用欧拉旋转,相机坐标系为参考坐标系，雷达坐标系为动坐标系,P_camera = T_l2c * P_lidar
 */
bool ExtriCal::calculateT(cv::Mat& _T, float _tran_x, float _tran_y, float _tran_z, float _roll, float _yaw, float _pitch){

    float r1 = cos(_roll)*cos(_yaw);
    float r2 = cos(_roll)*sin(_yaw)*sin(_pitch) - sin(_roll)*cos(_pitch);
    float r3 = cos(_roll)*sin(_yaw)*cos(_pitch) + sin(_roll)*sin(_pitch);
    float r4 = sin(_roll)*cos(_yaw);
    float r5 = sin(_roll)*sin(_yaw)*sin(_pitch) + cos(_roll)*cos(_pitch);
    float r6 = sin(_roll)*sin(_yaw)*cos(_pitch) - cos(_roll)*sin(_pitch);
    float r7 = -sin(_yaw);
    float r8 = cos(_yaw)*sin(_pitch);
    float r9 = cos(_yaw)*cos(_pitch);

    _T = (cv::Mat_<double>(4, 4) << r1, r2, r3, _tran_x, r4, r5, r6, _tran_y, r7, r8, r9, _tran_z, 0, 0, 0, 1);

    return true;
}

/**
 * @brief 将点云投影到像素
 */
bool ExtriCal::projectL2C(cv::Point3f _3d_point, cv::Point2f& _pixel_point, cv::Mat _T){
    //将3x1的点云，转换为4x1的世界坐标，因为外参是4x4
    cv::Mat world_point = cv::Mat(4,1,CV_64FC1);
    world_point.at<double>(0,0) = _3d_point.x * 1000;   //所有都在mm量级下计算
    world_point.at<double>(1,0) = _3d_point.y * 1000;
    world_point.at<double>(2,0) = _3d_point.z * 1000;
    world_point.at<double>(3,0) = 1;
    if(calib::Param::DEBUG_SHOW_COORDINATE == 1){
        cout << "--------point in world--------" << endl;
        cout << world_point << endl;
    }

    //由外参得到4x1的相机坐标系下坐标
    cv::Mat camera_point = cv::Mat(4,1,CV_64FC1);
    camera_point = _T * world_point;
    if(calib::Param::DEBUG_SHOW_COORDINATE == 1){
        cout << "--------point in camera--------" << endl;
        cout << camera_point << endl;
    }

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
    if(calib::Param::DEBUG_SHOW_COORDINATE == 1){
        //cout << "----------matrix_3x4_c2p--------" << endl;
        //cout << matrix_3x4_c2p << endl;
    }

    //由3x4的变换矩阵得到3x1的像素平面内的齐次坐标
    cv::Mat pixel_point = cv::Mat(3,1,CV_64FC1);
    pixel_point = matrix_3x4_c2p * camera_point / camera_point.at<double>(2,0);//除Z(深度)是得到归一化平面的坐标
    if(calib::Param::DEBUG_SHOW_COORDINATE == 1){
        //cout << "--------point in image(3x1)--------" << endl;
        //cout << pixel_point << endl;
    }

    //赋值到Point2f的像素坐标
    _pixel_point = cv::Point2f(pixel_point.at<double>(0,0),pixel_point.at<double>(1,0));
    if(calib::Param::DEBUG_SHOW_COORDINATE == 1){
        cout << "--------point in image--------" << endl;
        cout << _pixel_point << endl;
    }

    return true;
}

/**
 * @brief 判断是否中靶
 */
int ExtriCal::judgeHit(cv::Point2f _pixel_point, vector<cv::Point2f> _roi_points){
    
    //cout << _pixel_point << endl;
    for(int i = 0; i < _roi_points.size(); i++){ 
        //cout << _roi_points[i] << endl;
        if((int)_pixel_point.x == (int)_roi_points[i].x){//坐标化为整形进行比较，否则浮点型比较太难达到了
            if((int)_pixel_point.y == (int)_roi_points[i].y){
                //cout << "Hit !" << endl;
                return 1;
            }else{
                continue;
            }
        }else{
            continue;
        }
    }
    return 0;
}