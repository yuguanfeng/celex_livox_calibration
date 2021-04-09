#include <opencv2/opencv.hpp>

using namespace std;

namespace calib{

    class Param{
        public:
            static int WIDTH;//图片宽度
            static int HEIGHT;//图片长度
            
            static cv::Mat camera_matrix;//内参
            static cv::Mat distort;//畸变
            static cv::Mat MAPX;//畸变参数
            static cv::Mat MAPY;//畸变参数

            //外参初始值(相机坐标系为参考坐标系，雷达坐标系为动坐标系)
            static float OFFSET_X;//沿Xc轴的偏移
            static float OFFSET_Y;//沿Yc轴的偏移
            static float OFFSET_Z;//沿Zc轴的偏移
            static float ANGLE_X;//绕Xc轴的旋转角度(roll)
            static float ANGLE_Y;//绕Yc轴的旋转角度(yaw)
            static float ANGLE_Z;//绕Zc轴的旋转角度(pitch)

            //外参上下限
            static float Y_UP;
            static float Y_DOWN;
            static float Z_UP;
            static float Z_DOWN;
            static float ANGLEY_UP;
            static float ANGLEY_DOWN;

            //迭代参数
            static int DATASET_NUMBER;//数据集数量
            static float STEP_OFFSET;//每次增加的平移量
            static float STEP_ANGLE;//每次增加的旋转角度值
            static float MAX_HIT_RATE;//上靶率的上限
            static int MAX_ITER_NUMBER;//迭代次数的上限

            //调试相关
            static int DEBUG_SHOW_IMAGE;      //显示图像
            static int DEBUG_SHOW_POINTCLOUD; //显示点云
            static int DEBUG_SHOW_COORDINATE; //显示投影时具体坐标
            static int DEBUG_PROCESS;         //显示处理数据相关
            static int DEBUG_CALCULATE;       //显示计算上靶率相关

    };
}

