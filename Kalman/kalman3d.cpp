#include "kalman3d.h"

//Kalman3d::PredictByKF(Point3d worldPoint,double time_stamp)
//{

//    Mat correct_state;
//    float dt=(float)(time_stamp-last_time)/1000.0;
//    last_time=time_stamp;
//    Mat measurement = (Mat_<float>(3,1)<<worldPoint.x,
//                                         worldPoint.y,
//                                         worldPoint.z);

//    return (Point3f)worldPoint;
//}

Kalman3d::Kalman3d()
{
    cout<<"Kalman3d init begin"<<endl;
    string file_path="../xml_path/Kalman3d.xml";
    FileStorage fs;
    fs.open(file_path,FileStorage::READ);
    while(!fs.isOpened()){
          cout<<"xml floading failed..."<<endl;
         fs.open(file_path,FileStorage::READ);
    }
    cv::Mat processNoise, measurementNoise;
    cv::Mat t;
    fs["kalman_Q"] >> processNoise;
    fs["kalman_R"] >> measurementNoise;
    fs["cam2gyro_offset"] >> cam2gyro_offset;
    fs["gun2cam_offset"] >> gun2cam_offset;
    fs.release();
    KF = make_unique<KalmanFilter>(stateNum, measureNum, 0);
    KF->processNoiseCov = processNoise;                 // Q 过程噪声
    KF->measurementNoiseCov = measurementNoise;         // R 测量噪声
    setIdentity(KF->errorCovPost,Scalar::all(1));       // P 卡尔曼增益
    setIdentity(KF->transitionMatrix,Scalar::all(1));   // F 状态转移矩阵
    KF->measurementMatrix = (Mat_<float>(measureNum,stateNum) << 1,0,0,         //H
                                                                 0,0,0,
                                                                 0,0,1,
                                                                 0,0,0,
                                                                 0,0,0,
                                                                 0,1,0);
}

//void Kalman3d::Kalman_init(double Q, double R)
//{
//    //勿用
//    setIdentity(KF->processNoiseCov, Scalar::all(Q));//系统噪声方差矩阵Q
//    setIdentity(KF->measurementNoiseCov, Scalar::all(R));//测量噪声方差矩阵R
//    setIdentity(KF->errorCovPost, Scalar::all(1));//后验错误估计协方差矩阵P

//}

void Kalman3d::initState(Point3f newWordPoint)
{

    KF->statePost = (cv::Mat_<float>(stateNum, 1) << newWordPoint.x,
                                                    0,
                                                    newWordPoint.y,
                                                    0,
                                                    newWordPoint.z,
                                                    0);
}

Point3f Kalman3d::PredictByKF(Point3d worldPoint, double time_stamp,double pitch_recv)
{
    Mat correct_state;
    if(abs(time_stamp - last_t)>6000)
        last_t=time_stamp;
    float dt = (float)(time_stamp - last_t)/1000.0;
    last_t = time_stamp;
    //cout<<"worldPoint:"<<worldPoint<<endl;
    Mat measureMent = (Mat_<float>(measureNum,1)<<worldPoint.x,
                                                   worldPoint.y,
                                                   worldPoint.z);
    KF->transitionMatrix.at<float>(0,1) = dt;
    KF->transitionMatrix.at<float>(2,3) = dt;
    KF->transitionMatrix.at<float>(4,5) = dt;
    KF->predict();

    correct_state = KF->correct(measureMent);
    cv::Point3f world_next = predictNextPoint(correct_state, pitch_time.y + dt + delay_time);
//    cout<<"hhh:"<<world_next.x - worldPoint.x<<" "<<world_next.y - worldPoint.y<<" "<<world_next.z - worldPoint.z<<" "<<endl;
//    cout<<"                                 dt:"<<dt<<endl;
    Point3f WorldPoint(correct_state.at<float>(0,0),correct_state.at<float>(2,0),correct_state.at<float>(4,0));
    return predictNextPoint(correct_state , dt+ delay_time + getflytime(pitch_recv,WorldPoint));

}

Point3f Kalman3d::predictNextPoint(Mat result, float time)
{
    float t=time;
    return Point3f(result.at<float>(0,0)+t*result.at<float>(1,0),
                   result.at<float>(2,0)+t*result.at<float>(3,0),
                   result.at<float>(4,0)+t*result.at<float>(5,0)
                   );
}


