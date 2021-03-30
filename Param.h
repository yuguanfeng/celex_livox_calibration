#include <opencv2/opencv.hpp>

using namespace std;

namespace calib{

    class Param{
        public:
            static int DATASET_NUMBER;//数据集数量
            static int GRAY_THRESHOLD;//灰度阈值
            static float POINTCLOUD_THRESHOLD;//点云阈值

            static int WIDTH;//图片宽度
            static int HEIGHT;//图片长度
            
            static cv::Mat camera_matrix;//内参
            static cv::Mat distort;//畸变
            static cv::Mat MAPX;//畸变参数
            static cv::Mat MAPY;//畸变参数

            //外参初始值
            static float OFFSET_X;//相机坐标系与雷达坐标系X轴的偏移
            static float OFFSET_Y;//相机坐标系与雷达坐标系Y轴的偏移
            static float OFFSET_Z;//相机坐标系与雷达坐标系Z轴的偏移
            static float ANGLE_X;//相机坐标系与雷达坐标系X轴的旋转角度
            static float ANGLE_Y;//相机坐标系与雷达坐标系Y轴的旋转角度
            static float ANGLE_Z;//相机坐标系与雷达坐标系Z轴的旋转角度

            //迭代参数
            static float STEP_OFFSET;//每次增加的平移量
            static float STEP_ANGLE;//每次增加的旋转角度值
            static float MAX_HIT_RATE;//上靶率的上限
            static int MAX_ITER_NUMBER;//迭代次数的上限

    };
}

