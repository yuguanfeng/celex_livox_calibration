#include <iostream>
#include "ExtriCal.h"
#include "common_PCL.h"

using namespace std;
using namespace cv;

#define DATASET_NUMBER 7
#define GRAY_THRESHOLD 50



int main(void){

    ExtriCal ex_cal;
    for (int i = 0; i < DATASET_NUMBER; i++){

        //读取图像，并处理得到白板所在像素集合
        string img_path = "./data/camera/" + i + ".bmp";
        Mat img = imread(img_path);
        cvtColor(img, img, COLOR_BGR2GRAY);   //测试时输入时rgb图像，先进行灰度转化，实际事件相机得到就是灰度图像
        ex_cal.processImage(img, ex_cal.roi_points, GRAY_THRESHOLD); 
        cout << "The number of roi_points in No." << i << "image is" << ex_cal.roi_points.size() << endl;

    }

}