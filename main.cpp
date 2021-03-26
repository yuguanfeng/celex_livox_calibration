#include <iostream>
#include <string.h>
#include "ExtriCal.h"
#include "common_PCL.h"

using namespace std;

#define DATASET_NUMBER 7
#define GRAY_THRESHOLD 155



int main(void){

    ExtriCal ex_cal;
    for (int i = 0; i < DATASET_NUMBER; i++){

        cout << "～～～～～～～～begin process image～～～～～～～～" << endl;
        //读取图像，并处理得到白板所在像素集合
        string img_path = "/home/yuguanfeng/celex_livox_calibration/data/camera/" 
                         + std::to_string(i) + ".bmp";    //注意要绝对路径，相对路径会报错
        cv::Mat img = cv::imread(img_path);
        if(img.empty()){
            cout << "Image loading failed !" << endl;
            return -1;
        }
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);   //测试时输入时rgb图像，先进行灰度转化，实际事件相机得到就是灰度图像
        if(ex_cal.processImage(img, ex_cal.roi_points, GRAY_THRESHOLD)){
            cout << "The number of roi_points in No." << i << " image is " << ex_cal.roi_points.size() << endl << endl;
        }else{
            cout << "Process image failed !" << endl;
            return -1;
        }

        
         
        

    }
    return 0;

}