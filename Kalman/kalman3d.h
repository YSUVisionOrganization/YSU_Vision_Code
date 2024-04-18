/*-----------------------------------------
 * Brief:使用世界坐标系进行卡尔曼滤波器的预测
 *
 *
 *-----------------------------------------*/
#ifndef KALMAN3D_H
#define KALMAN3D_H
#include "Main/headfiles.h"

class Kalman3d{
private:
    unique_ptr<KalmanFilter> KF;//unique需要用std::move进行传参
    const int measureNum = 3;
    const int stateNum = 6;
    double bullet_speed = 15.0;//弹丸速度
    uint32_t last_t = 0;

    double delay_time = 0.319; // 发弹延迟、通信延迟等较为固定延迟 2是极限
    double k = 0.0402;         // 阻力系数
    double g = 9.75;           // 重力加速度
    int iter_num = 40;

    double bs_coeff = 0.9; // 初始弹速系数
public:
//公共参数:
    Kalman3d();
    void Kalman_init(double Q,double R);
    void initState(Point3f newWordPoint);


    cv::Point3f cam2gyro_offset; // 陀螺仪到相机的平移偏移量
    cv::Point3f gun2cam_offset;  // 枪口相对于相机的平移偏移量
    cv::Point2f pitch_time;      // 弹道补偿抬枪角度以及子弹飞行时间
    cv::Point3f world_coord;
    cv::Point3f correct_coord;

//公共函数:
    Point3f predictNextPoint(Mat result,float time);
    Point3f PredictByKF(Point3d worldPoint,double time_stamp,double pitch_recv);

        /**
         *  @brief  计算子弹飞行时间
         *  @param  angle   pitch角
         *  @param  correct_state   KF得出的状态值
         *  @param  T   n-1状态中子弹飞行时间
         *  @param  dt  图片处理延迟
         *  @return 返回子弹在绝对坐标系中飞行的时间；
         */
        double getflytime(double angle, cv::Point3f correct_spin)
        {
            double x = sqrt(correct_spin.z * correct_spin.z + correct_spin.y * correct_spin.y) / 1000.0;
            angle=angle / 180 * CV_PI;
            return (exp(k * x) - 1.0) / (k * bs_coeff * bullet_speed * cos(angle)); //- dt - delay_time;
        }
};



#endif // KALMAN3D_H
