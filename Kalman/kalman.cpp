#include "kalman.h"
#include "Communication/communication.h"
#include "ThreadManager/thread_manager.h"
#include"Forecast/forecast.h"

double trans_pretime_x=1;
double trans_pretime_y=1; // 越大越飘,决定超前回归的速度（正相关），这两个量会影响彼此
int ant_if1=900;
int ant_if2=1;
#define bullet_speed 100//子弹速度，单位未定
#define delay_time 100//延迟时间，自设，单位未定
Kalman::Kalman():
    KF(make_unique<KalmanFilter>(stateNum, measureNum, 0)),
     anti_factor(2)//提前的倍数
{}

//
///
/// \brief 第二个数越大约不信任观测值,越滞后
/// \param Q
/// \param R
///
void Kalman::Kalman_init(double Q,double R)//Kalman_init(double Q=0.00001,double R=0.1)
/*void Kalman_init(KalmanFilter* KF,double Q,double R)*/{
        KF->transitionMatrix = (Mat_<float>(4, 4) <<
                                1, 0, trans_pretime_y, 0,
                                0, 1, 0, trans_pretime_x,
                                0, 0, 1, 0,
                                0, 0, 0, 1);  //转移矩阵A，第3个数字和8个数字决定跟随的程度，越大跟的越快
        //setIdentity是创建单位矩阵的，后面不带Scalar参数表示默认创建单位矩阵
        setIdentity(KF->measurementMatrix);                                             //测量矩阵H
        setIdentity(KF->processNoiseCov, Scalar::all(Q)); //0.00001                   //系统噪声方差矩阵Q
        setIdentity(KF->measurementNoiseCov, Scalar::all(R)); //0.1                   //测量噪声方差矩阵R
        setIdentity(KF->errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
        Kalman::measurement =Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义
        is_jump=false;
        last_point=Point(0,0);
        //Kalman::measurement.at<float>(0) = first_x;
        //Kalman::measurement.at<float>(1) = first_y;
//        cout<<"     kalman init finish"<<endl;
}

Point Kalman::Kalman_filter(Point measure_point,double jump_factor,double ant_closed)
/*Point Kalman_filter(KalmanFilter* KF, Point measure_point)*/{

        //2.kalman prediction
        Mat prediction =KF->predict();
        Point predict_pt = Point(prediction.at<float>(0), prediction.at<float>(1));   //预测值(x',y')
        //3.update measurement
        Kalman::measurement.at<float>(0) = (float)measure_point.x;
        Kalman::measurement.at<float>(1) = (float)measure_point.y;
        //4.update
        KF->correct(Kalman::measurement);
        //Point correct_pt =predict_pt;
        Point correct_pt = Point(KF->statePost.at<float>(0), KF->statePost.at<float>(1));   //预测值(x',y')
        //return Kalman_predict();


        Point anti_kalmanPoint;
        if((measure_point.x + jump_factor*(measure_point.x - correct_pt.x))<=ant_if1
            ||(measure_point.x + jump_factor*(measure_point.x - correct_pt.x))>=ant_if2)//Prevent Anti-kal out of Mat
        {

            if(abs(measure_point.x - correct_pt.x) > ant_closed)//When points are closed, no Anti-kalman to reduce shaking
            {
                anti_kalmanPoint.x = measure_point.x + jump_factor*(measure_point.x - correct_pt.x);}

            else
            {
                anti_kalmanPoint.x = measure_point.x;
            }
        }
        else
        {

            anti_kalmanPoint.x = measure_point.x;
        }

        if((measure_point.y + jump_factor*(measure_point.y - correct_pt.y))<=ant_if1
            || (measure_point.y + jump_factor*(measure_point.y - correct_pt.y))>=ant_if2)//Prevent Anti-kal out of Mat
        {
            if(abs(measure_point.y - correct_pt.y) > ant_closed)//When points are closed, no Anti-kalman to reduce shaking
                anti_kalmanPoint.y = measure_point.y +jump_factor*(measure_point.y - correct_pt.y);
            else
                anti_kalmanPoint.y = measure_point.y;
        }
        else
        {
            anti_kalmanPoint.y = measure_point.y;
        }
        last_point=anti_kalmanPoint;

        return  anti_kalmanPoint;
}

Point2f Kalman::only_Kalman_filter(Point2f measure_point)
{
    Mat prediction =KF->predict();
//    Point2f predict_pt = Point(prediction.at<float>(0), prediction.at<float>(1));   //预测值(x',y')

//    KF->transitionMatrix.at<float>(0,1)=dt;
//    KF->transitionMatrix.at<float>(2,3)=dt;
    //3.update measurement
    Kalman::measurement.at<float>(0) = (float)measure_point.x;
    Kalman::measurement.at<float>(1) = (float)measure_point.y;
    //4.update
    Mat correct = KF->correct(Kalman::measurement);
    //[0]+t*[1]=pre_x,
//    [2]+t*[3]=pre_y
    //Point correct_pt =predict_pt;
    Point2f correct_pt = Point2f(KF->statePost.at<float>(0), KF->statePost.at<float>(1));   //预测值(x',y')
    return correct_pt;
}
//Point3d Kalman::Kalman_filter_3d(Point3d measure_point,Forecast& test_zhizhen)
//{
//    double last_x=0;
//    double last_y=0;
//    double last_z=0;
//    Mat correct_state;
//    double last_time=0;
//    double t_22=test_22_t(test_zhizhen);
//    Point3d coord_now=test_22_coord(test_zhizhen);
//    double now_x=coord_now.x;
//    double dx=coord_now.x-last_x;
//    double dy=coord_now.y-last_y;
//    double dz=coord_now.z-last_z;
////    Point3d d_coord=Point3d(dx,dy,dz);//d位移，                                           用于求图像中的速度
//    float dt=(float)(t_22-last_time);
//    //    v=dx/dt
//    double v_x=dx/dt;
//    double v_y=dy/dt;
//    double v_z=dz/dt;
//    Point3d v_k=Point3d (v_x,v_y,v_z);

//    last_x=coord_now.x;
//    last_y=coord_now.y;
//    last_z=coord_now.z;


//    last_time=t_22;
////    Mat prediction =KF->predict();
////    Point2f predict_pt = Point(prediction.at<float>(0), prediction.at<float>(1));   //预测值(x',y')


//    //3.update measurement
//    Kalman::measurement3d.at<double>(0) = (double)measure_point.x;
//    Kalman::measurement3d.at<double>(1) = (double)measure_point.y;
//    Kalman::measurement3d.at<double>(2) = (double)measure_point.z;

//        KF->transitionMatrix.at<float>(0,1)=dt;
//        KF->transitionMatrix.at<float>(2,3)=dt;
//        KF->transitionMatrix.at<float>(4,5)=dt;
//        KF->predict();

//    //4.update
//    correct_state=KF->correct(measurement3d);
//    Point3d correct_coord= Point3d(correct_state.at<float>(0,0),correct_state.at<float>(2,0),correct_state.at<float>(4,0));
//    double t_fly=correct_state.at<float>(4,0)/bullet_speed;//z坐标除以子弹速度
//    //Point correct_pt =predict_pt;
////    Point3d correct_pt = Point3d(KF->statePost.a t<float>(0), KF->statePost.at<float>(1),KF->statePost.at<float>(2));   //预测值(x',y')
//    Point3d next_22=predictNextPoint(correct_state,t_fly+dt+delay_time,v_k);
//    return next_22;
//}
//Point3d Kalman::predictNextPoint(Mat result,float time,Point3d v_)    //[0]+t*[1]=pre_x      [2]+t*[3]=pre_y
//{
//    float t=time;
//    float x=result.at<float>(0,0) + t * v_.x;
//    float y=result.at<float>(2,0) + t * v_.y;
//    float z=result.at<float>(4,0) + t * v_.z;

////    float x=result.at<float>(0,0) + t * result.at<float>(1,0);
////    float y=result.at<float>(2,0) + t * result.at<float>(3,0);
////    float z=result.at<float>(4,0) + t * result.at<float>(5,0);
//    return Point3d(x,y,z);


//}
//
//  * @name   kalmanCreate
//  * @brief  ����һ���������˲���
//  * @param  p:  �˲���
//  *         T_Q:ϵͳ����Э����
//  *         T_R:��������Э����
//  *
//  * @retval none
//  * @attention 	R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
//  *		       		��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
//  */
//void Kalman_t::KalmanInit(double T_Q,double T_R)
//{
//    X_last = (double)0;
//    P_last = 0;
//    Q = T_Q;
//    R = T_R;
//    A = 1;
//    B = 0;
//    H = 1;
//    X_mid = X_last;
//}


//  * @name   KalmanFilter
//  * @brief  �������˲���
//  * @param  p:  �˲���
//  *         dat:���˲�����
//  * @retval �˲��������
//  * @attention Z(k)��ϵͳ����,������ֵ   X(k|k)�ǿ������˲����ֵ,���������
//  *            A=1
//        B=0 H=1 I=1  W(K)  V(k)�Ǹ�˹������,�����ڲ���ֵ����,���Բ��ù�
//  *            �����ǿ�������5�����Ĺ�ʽ
//  *            һ��H'��Ϊ������,����Ϊת�þ���
//  */

//double Kalman_t::KalmanFilter(double dat)
//{
//    X_mid =A*X_last;                     //�ٶȶ�Ӧ��ʽ(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
//    P_mid = A*P_last+Q;               //�ٶȶ�Ӧ��ʽ(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
//    kg = P_mid/(P_mid+R);             //�ٶȶ�Ӧ��ʽ(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
//    X_now = X_mid+kg*(dat-X_mid);     //�ٶȶ�Ӧ��ʽ(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
//    P_now = (1-kg)*P_mid;                //�ٶȶ�Ӧ��ʽ(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
//    P_last = P_now;                         //״̬����
//    X_last = X_now;
//    return X_now;							  //���Ԥ����x(k|k)
//}
