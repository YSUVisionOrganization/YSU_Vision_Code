#ifndef ANGLESOLVER_H
#define ANGLESOLVER_H
#define LONG_CAM_TO_CENTER 92.3 //相机到轴长度
#include "Main/headfiles.h"
#include "GafSolver/iterative_projectile_tool.h"
#include "GafSolver/gimbal_transform_tool.h"
#include "GafSolver/gaf_projectile_solver.h"
#include "Kalman/kalman.h"
#define GRACOMP_DIS  7500 //gravity compensation velocity threshold
#define BULLETFIRE_V 1000 //bullet firing velocity
#define CAM_SUPPORT_DIS 2450 //238mm
class AngleSolver
{
public:
    AngleSolver();
    void InitAngle();
    double * SolveAngle(vector<cv::Point2f>& object_armor_points_,double y_p_recv[4]);
    int shoot_get();

    vector<cv::Point3f> obj;
    cv::Mat rVec;
    cv::Mat tVec;
    cv:: Mat cam;
    cv:: Mat disCoeffD;
    int shoot=0;
    float gra_t;
    double _xErr;
    double _yErr;
    double distance_3d;
    double p_y_err[2];
    void P4P_solver();
    void gravity_comp();
//    Point2f trajectory_simulation(Point2f aim,double v0,double angle); //弹道模拟
//    double gravity_compensation(Point2f aim, double v0);//重力补偿
    Point2f canHitTarget(Point2f aim, double v0, double angle);
    double calculateLaunchAngle(Point2f aim,double v0);


    void changeYaw(Point3d& point, double yaw);
    void changePitch(Point3d& point, double pitch);
    void Camara_to_World(Point3d& point,double yaw,double pitch);
    void recoverYaw(Point3d& point, double yaw);
    void recoverPitch(Point3d& point, double pitch);
    void World_to_Camara(Point3d& point,double yaw,double pitch);

    Point3d SolveWorldPoint(vector<cv::Point2f>& object_armor_points_,double y_p_recv[4]);//返回装甲板中心点在世界坐标系下的三维坐标
    double* WorldToAngle(const Point3d& WordForcast,double y_p_recv[4]);


private:
    float y_yaw;
    float x_pitch;
    Kalman kalman_p;
    std::shared_ptr<rmoss_projectile_motion::GafProjectileSolver> gaf_solver; // 重力补偿
    std::shared_ptr<rmoss_projectile_motion::GimbalTransformTool> projectile_tansformoss_tool; // 重力补偿

    double cam2center[3]; // LONG_CAM_TO_CENTER
    Point3d zeroPoint;


};

#endif // ANGLESOLVER_H
