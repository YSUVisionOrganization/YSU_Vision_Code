#ifndef FORECAST_H
#define FORECAST_H

#include "Main/headfiles.h"//
#include "Kalman/kalman.h"
#include "Forecast/ysu_gsl.h"
#include "Pose/angle_solver.h"


struct chuo//li shi chuo
{
    vector<Point2f> rect;
    Point2f center;
    double time;
    chuo(){};
    chuo(vector<Point2f> &ora,double &time_):rect(ora),time(time_),center(Point2f((ora[0].x+ora[1].x+ora[2].x+ora[3].x)/4,(ora[0].y+ora[1].y+ora[2].y+ora[3].y)/4)){};    
};

struct chuo3d
{
    Point3d center;
    double time;
    chuo3d(){};
    chuo3d(Point3d &center_,double &time_ ):center(center_),time(time_){};
};

struct PYT{
    double pitch;
    double pitch_speed;
    double yaw;
    double yaw_speed;
    double time;
    PYT(){}
    PYT(double *yp_real,double *y_p_recv,double &time_):pitch(yp_real[1]),pitch_speed(y_p_recv[0]),yaw(yp_real[0]),yaw_speed(y_p_recv[2]),time(time_){}
};

class Forecast
{
public:

    Forecast();
    void Init();
    vector<Point2f>& forcast(vector<Point2f> &original,double time,double y_p_recv[4]);
    Point2f lu, ld, ru, rd;
    double* angle_forcast(double* y_p_err,double* y_p_recv,double time,int shoot);
    Point3d WorldForcast(Point3d original,double sys_time,double pitch_recv);

    chuo3d now3d;
    vector<chuo3d> record_history3d;


private:

    void get_forecast();
    double my_gsl(data d, double aim_time);
  //  bool lagrangeint(vector<double>&X, vector<double>&Y, vector<double>&xp, vector<double> &get);
    chuo now;


    vector<chuo> record_history;

    vector<Point2f> result;//输出的装甲板四个点（预测后的值）

    vector<Point2f>  last_result;

    int record_history_size;

    int record_history_interval_max;
    int recording_interval;

    double pre_time;


    int lost_aim_max;
    int lost_aim_num;

    vector<double> pitch_history;
    // 各结果混合赋权滤波输出，如果单纯预测结果足够逼近实际且顺滑，则此部不需要。
    // double real_weight;
    // double fore_weight;
    // double last_result_weight;
    // double Kalman_Q,Kalman_R;
    //  vector<unique_ptr<Kalman_t>> p_kal;
    Point3d result_center3d;
    Point2f result_center;
    float vec_result_to_limited[2];
    float result_correct_ratio;
    double original_vec_length;
    double vec_distribution[8];

    ysugsl* p_gsl[2];

    double *ft[3];
    data d[3];
    double *weight[2];//预测历史帧权重。
    double *t;

    vector<Point2f> centers;//用于卡尔曼计算速度

    vector<PYT> angle_history;
    double y_p_real;

    double yp_real[2];
    PYT pynow;
    Point2f result_py;
    double pitch;

};


#endif // FORECAST_H
