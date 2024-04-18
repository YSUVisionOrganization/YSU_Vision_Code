#include "Main/headfiles.h"
#include "Pose/angle_solver.h"
#include <Kalman/kalman.h>
#define DEBUG // 打开调试模式，输出yam轴pitch轴误差
#define LONG_CAM_TO_CENTER 92.3 //相机到轴长度

AngleSolver::AngleSolver()
{

}

void AngleSolver::InitAngle()
{
    cam=Mat(3,3,CV_32FC1,Scalar::all(0));
    disCoeffD=Mat(1,5,CV_32FC1,Scalar::all(0));

    string file_path="../xml_path/camera.xml";
    FileStorage fr;
    fr.open(file_path,FileStorage::READ);
    while(!fr.isOpened()){
        cout<<"camera xml floading failed..."<<endl;
        fr=FileStorage(file_path,FileStorage::READ);
    }
    fr["camera-matrix"]>>cam;
    fr["distortion"]>>disCoeffD;
    fr.release();

    obj = vector<Point3f>{
            cv::Point3f(-61.5f,-30.0f,0),
            cv::Point3f(61.5f,-30.0f,0),
            cv::Point3f(61.5f,30.0f,0),
            cv::Point3f(-61.5f,30.0f,0)
    };
    rVec = cv::Mat::zeros(3,1,CV_64FC1);
    tVec = cv::Mat::zeros(3,1,CV_64FC1);
    // 迭代法重力补偿初始化
    gaf_solver = std::make_shared<rmoss_projectile_motion::GafProjectileSolver>(25, 0.01);
    projectile_tansformoss_tool = std::make_shared<rmoss_projectile_motion::GimbalTransformTool>(gaf_solver);
    kalman_p.Kalman_init();
    cam2center[0]=cam2center[1]=cam2center[2]=0;

    cam2center[2]=LONG_CAM_TO_CENTER;
    zeroPoint=Point3d(0,0,0);
}

//电控传入的是角度制，pitch正方向沿y轴正方向，抬头算负数
//double * AngleSolver::SolveAngle(vector<Point2f>& object_armor_points_,double (y_p_recv[4]))
//{
//    if(!(object_armor_points_[0] == Point2f(0, 0) && object_armor_points_[1] == Point2f(0, 0) && object_armor_points_[2] == Point2f(0, 0) && object_armor_points_[3] == Point2f(0, 0)))
//    {
//        cout<<"out of size:"<<object_armor_points_[0]<<","<<object_armor_points_[1]<<","<<object_armor_points_[2]<<","<<object_armor_points_[3]<<endl;

//    cv::solvePnP(obj,object_armor_points_,cam,disCoeffD,rVec,tVec,false,SOLVEPNP_ITERATIVE);

//    _xErr = atan(tVec.at<double>(0, 0) / tVec.at<double>(2, 0)) / 2 / CV_PI * 360;//+5
//    _yErr = atan(tVec.at<double>(1, 0) / tVec.at<double>(2, 0)) / 2 / CV_PI * 360;//atan求出来的是弧度制,这里已经改成角度制了

//    p_y_err[0] = _xErr;//xyErr都是角度制
//    p_y_err[1] = _yErr;
//    //求距离



//    double x_pos=tVec.at<double>(0,0)/1000;
//    double y_pos=tVec.at<double>(1,0)/1000;
//    double z_pos=tVec.at<double>(2,0)/1000 +LONG_CAM_TO_CENTER ;
//    Point3d point(x_pos,y_pos,z_pos);

//    Camara_to_World(point, y_p_recv[0] , y_p_recv[2]);

//    distance_3d=sqrt(point.x*point.x+point.y*point.y+point.z*point.z);
//    gra_t=distance_3d/BULLETFIRE_V;
//    shoot=1;

//    //5.23新的重力补偿  -> 0   往上-90 往下+90 角度制
//    float aim_angle = (y_p_recv[2]+_yErr)*CV_PI/180;
//    Point2f aim_pos = {(float)(cos(aim_angle)*distance_3d),(float)(sin(aim_angle)*distance_3d)};//sincos的参数是弧度制
//    //补偿后的角度
//    //double comp_angle=aim_angle;
//    //double delta_angle=calculateLaunchAngle(aim_pos,20)*180/CV_PI-p_y_err[1];//小弹丸射速哨兵20.5m/s步兵23.5m/s   英雄为20   //*****************************调初速度

//    p_y_err[0] = p_y_err[0] ;//-7.8 ;//步兵yaw增加弹丸向右偏
//    p_y_err[1]=_yErr ;//pitch,哨兵+0.5，步

//    }
//    else
//    {
//        p_y_err[0] = 0;
//        p_y_err[1] = 0;//top_min=-19
//        shoot=0;
//    }

//    return p_y_err;
//}
Point3d AngleSolver::SolveWorldPoint(vector<Point2f> &object_armor_points_, double y_p_recv[])
{   
    Point3d point(0,0,0);
    if(!(object_armor_points_[0] == Point2f(0, 0) && object_armor_points_[1] == Point2f(0, 0) && object_armor_points_[2] == Point2f(0, 0) && object_armor_points_[3] == Point2f(0, 0)))
    {

        cv::solvePnP(obj,object_armor_points_,cam,disCoeffD,rVec,tVec,false,SOLVEPNP_ITERATIVE);
        _xErr = atan(tVec.at<double>(0, 0) / tVec.at<double>(2, 0)) / 2 / CV_PI * 360;//+5
        _yErr = atan(tVec.at<double>(1, 0) / tVec.at<double>(2, 0)) / 2 / CV_PI * 360;//atan求出来的是弧度制,这里已经改成角度制了

        p_y_err[0] = _xErr;//xyErr都是角度制
        p_y_err[1] = _yErr;

        //求距离
        double x_pos=tVec.at<double>(0,0)+cam2center[0];
        double y_pos=tVec.at<double>(1,0)+cam2center[1];
        double z_pos=tVec.at<double>(2,0)+cam2center[2];  //+LONG_CAM_TO_CENTER ;
        point = Point3d(x_pos,y_pos,z_pos);
        Camara_to_World(point, y_p_recv[0] , y_p_recv[2]);

#ifdef DEBUG
        double step=1;
        char c=waitKey(1);
        if(c==82){
            cam2center[2]+=step;
        }
        else if(c==84){
            cam2center[2]-=step;
        }
        Mat canvas(260,800, CV_8UC3, Scalar(0, 0, 0));
        String s1="_xErr:"+to_string(_xErr)+",_yErr:"+to_string(_yErr)+"  "+to_string(z_pos);
        String s2="x_pos:"+to_string(x_pos)+",y_pos:"+to_string(y_pos)+",z_pos:"+to_string(z_pos);
        String s3="c:"+to_string(c)+",zeroPoint:"+to_string((int)zeroPoint.x)+" "+to_string((int)zeroPoint.y)+" "+to_string((int)zeroPoint.z);

        String s4="point[0]:"+to_string(point.x)+",point[1]"+to_string(point.y)+",point[2]:"+to_string(point.z);
        String s5="y_p_recv[0]"+to_string(y_p_recv[0])+",y_p_recv[2]:"+to_string(y_p_recv[2]);
        String s6="cam2center:"+to_string(cam2center[0])+":"+to_string(cam2center[1])+":"+to_string(cam2center[2]);

        putText(canvas,s1,Point(10,40),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,255,255),2);
        putText(canvas,s2,Point(10,80),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,255,255),2);
        putText(canvas,s3,Point(10,120),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,255,255),1);
        putText(canvas,s4,Point(10,160),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,255,255),2);
        putText(canvas,s5,Point(10,200),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,255,255),2);
        putText(canvas,s6,Point(10,240),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,255,255),1);

        imshow("SolveWordPoint",canvas);
#endif



    }
    else
    {
        shoot=0;
    }


    return point;
}

double *AngleSolver::WorldToAngle(const Point3d &WorldForcast,double y_p_recv[4])
{
    if(!(WorldForcast.x==0&&WorldForcast.y==0&&WorldForcast.z==0))
    {
        Point3d CamForcast=WorldForcast;
        World_to_Camara(CamForcast,y_p_recv[0] , y_p_recv[2]);

        CamForcast.x-=cam2center[0];
        CamForcast.y-=cam2center[1];
        CamForcast.z-=cam2center[2];

        //设置偏置
        p_y_err[0] = atan(CamForcast.x/CamForcast.z) / CV_PI * 180 -3.0;
        p_y_err[1] = atan(CamForcast.y/CamForcast.z) / CV_PI * 180 +1.0;

        distance_3d=sqrt(CamForcast.x*CamForcast.x+CamForcast.y*CamForcast.y+CamForcast.z*CamForcast.z)/1000.0;
        float angle=(y_p_recv[2]+p_y_err[1])*CV_PI/180;
        Point2f position = {(float)(cos(angle)*distance_3d),(float)(sin(angle)*distance_3d)};
        p_y_err[1]=calculateLaunchAngle(position,30)*180/CV_PI-y_p_recv[2];//重力补偿
        shoot=1;


#ifdef DEBUG
        Mat canvas(200,800, CV_8UC3, Scalar(0, 0, 0));
        String s1="yaw:"+to_string(p_y_err[0] )+",pitch:"+to_string(p_y_err[1] );
        String s2="position:"+to_string(position.x)+",angle:"+to_string(position.y);
        String s3="World.x:"+to_string(WorldForcast.x)+",CamForcast.y"+to_string(WorldForcast.y)+"World.z:"+to_string(WorldForcast.z);
        String s4="y_p_recv[0]:"+to_string(y_p_recv[0])+",y_p_recv[2]"+to_string(y_p_recv[2]);

        putText(canvas,s1,Point(10,40),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,255,255),2);
        putText(canvas,s2,Point(10,80),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,255,255),2);
        putText(canvas,s3,Point(10,120),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,255,255),2);
        putText(canvas,s4,Point(10,160),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,255,255),2);

        imshow("WordToAngle",canvas);
#endif
    }
    else
    {
        p_y_err[0] = 0;
        p_y_err[1] = 0;
        shoot=0;
    }

    return p_y_err;
}





void AngleSolver::changeYaw(Point3d &point, double yaw){
    //Ry
    double cosYaw = cos(yaw);
    double sinYaw = sin(yaw);
    double x = point.x *cosYaw - point.z * sinYaw;
    double z = point.x *sinYaw + point.z * cosYaw;
    point.x = x;
    point.z = z;
}
void AngleSolver::changePitch(Point3d &point, double pitch){
    double cosPitch = cos(pitch);
    double sinPitch = sin(pitch);
    double y = point.z *sinPitch + point.y * cosPitch;
    double z = point.z *cosPitch - point.y * sinPitch;

    point.y = y;
    point.z = z;
}
///
/// \brief point:实参传递，相机坐标系点->世界坐标系点
/// \param point:实参传递，相机坐标系点->世界坐标系点
/// \param yaw:云台yaw角度
/// \param pitch:云台pitch角度
///
void AngleSolver::Camara_to_World(Point3d &point, double yaw, double pitch){
    //自带的cos和sin函数输入的数据应该是弧度制数据，但是电控传过来的是角度制，所以在此处进行转换
    pitch=pitch*CV_PI/180;
    yaw=yaw*CV_PI/180;
    changeYaw(point,yaw);
    changePitch(point,pitch);
}
void AngleSolver::recoverYaw(Point3d &point, double yaw){
    double cosYaw = cos(2*CV_PI-yaw);
    double sinYaw = sin(2*CV_PI-yaw);
    double x = point.x *cosYaw - point.z * sinYaw;
    double z = point.x *sinYaw + point.z * cosYaw;
    point.x = x;
    point.z = z;
}
void AngleSolver::recoverPitch(Point3d &point, double pitch){
    double cosPitch = cos(2*CV_PI-pitch);
    double sinPitch = sin(2*CV_PI-pitch);
    double y = point.z *sinPitch + point.y * cosPitch;
    double z = point.z *cosPitch - point.y * sinPitch;
//    double y = point.z *sinPitch + point.y * cosPitch;
//    double z = point.z *cosPitch - point.y * sinPitch;
    point.y = y;
    point.z = z;
}
void AngleSolver::World_to_Camara(Point3d &point, double yaw, double pitch){
    //自带的cos和sin函数输入的数据应该是弧度制数据，但是电控传过来的是角度制，所以在此处进行转换
    pitch=pitch*CV_PI/180;
    yaw=yaw*CV_PI/180;
    recoverPitch(point,pitch);
    recoverYaw(point,yaw);

}


int AngleSolver::shoot_get()
{
    return shoot;
}

void AngleSolver::P4P_solver()
{
    double x_pos = tVec.at<double>(0, 0);
    double y_pos = tVec.at<double>(1, 0);
    double z_pos = tVec.at<double>(2, 0);

    double tan_pitch = y_pos / sqrt(x_pos*x_pos + z_pos * z_pos);
    double tan_yaw = x_pos / z_pos;
    x_pitch = -atan(tan_pitch) * 180 / CV_PI;
    y_yaw = atan(tan_yaw) * 180 / CV_PI;


}

void AngleSolver::gravity_comp()
{


//下面是旧版的重力补偿
    gra_t=distance_3d/BULLETFIRE_V;
    if(distance_3d>GRACOMP_DIS)
    {
        p_y_err[0] = _xErr;
//       p_y_err[1] = _yErr;
        p_y_err[1] = _yErr-(CAM_SUPPORT_DIS+4.9*gra_t*gra_t)*0.001;
        //p_y_err[1] = _yErr-(BULLETFIRE_V*(distance_3d/BULLETFIRE_V)*(CAM_SUPPORT_DIS/distance_3d)+0.5*9.8*gra_t*gra_t);

    }
    else
    {
        p_y_err[0] = _xErr;
        p_y_err[1] = _yErr;
    }
}


Point2f AngleSolver::canHitTarget(Point2f aim, double v0, double angle) {

#define T 0.01
#define G 9.81
#define K 6.877412475e-05 //0.0003126050550//
#define epsilon  0.001
#define m 0.0033    //0.045//

    Point2f v = { (float)(v0 * cos(angle)),(float)(v0 * sin(angle)) };
    Point2f a;
    Point2f p = { 0,0 };
    double v_total;

    while (p.x<aim.x && v.x>epsilon ) {
        v_total = sqrt(v.x * v.x + v.y * v.y);
        a.x = (-K * v.x * v_total)/m;
        a.y = (-K * v.y * v_total - m*G)/m;

        v += a * T;
        p += v * T + 0.5 * a * T * T;
        cout<<"aimx"<<aim.x<<" //aimy"<<aim.y<<" //px"<<p.x<<" //py"<<p.y<<" //vx"<<v.x<<" //vy"<<v.y<<" //ax"<<a.x<<" //ay"<<a.y<<endl;


    }
    return {0,p.y - aim.y };


}

double AngleSolver::calculateLaunchAngle(Point2f aim,double v0) {
    double lowAngle = 0;//最小的发射角0.0
    double highAngle = CV_PI / 2;//最大的发射角
    double launchAngle = 0.0;//当前发射角
    int maxIterations = 15; // 最大迭代次数
    int iterations = 0; // 当前迭代次数
    Point2f dp;
    Point2f temp_aim = aim;


    while (highAngle - lowAngle > epsilon && iterations <= maxIterations) {
        launchAngle = (lowAngle + highAngle) / 2;

        cout<<"launchAngle1:"<<launchAngle/CV_PI*180<<endl;

        dp = canHitTarget(temp_aim, v0, launchAngle);

        if (dp.y>0) {
            highAngle = launchAngle;
        }
        else {
            lowAngle = launchAngle;
        }
        iterations++;
        cout<<"++"<<endl;
        cout<<"launchangle:"<<launchAngle/CV_PI*180<<endl;

    }
    if (iterations > maxIterations) {
        shoot = 0;// 在达到最大迭代次数时，返回一个特定的错误值或抛出异常,这里就是不开火
        return -2024;// 返回一个无效的角度值或抛出一个适当的异常
    }
    shoot = 1;
    return launchAngle;//准许开火
}
