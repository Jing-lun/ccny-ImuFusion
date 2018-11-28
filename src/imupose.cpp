#include "imupose.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>

Imuposeinf::Imuposeinf()
{
    //变量初始化
    mAdd_w << 0,0,0;
    mAdd_a << 0,0,0;
    mIMUOriCount = false;

    gn << 0, 0, -9.7919;
    ba << 0.01, 0.01, 0.01;
    bw << 0.0016, 0.0013, 0.0024;//0.0016, 0.0013, 0.0024

    q_pre.w() = 1.0;
    q_pre.x() = 0.0;
    q_pre.y() = 0.0;
    q_pre.z() = 0.0;
    Cbn_pre << 1,0,0,0,1,0,0,0,1;

    q_now.w() = 1.0;
    q_now.x() = 0.0;
    q_now.y() = 0.0;
    q_now.z() = 0.0;

    V_pre << 0, 0, 0;
    V_now << 0, 0, 0;
    P_pre << 0, 0, 0;
    P_now << 0, 0, 0;

    n_a << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
    n_ba << 0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001;
    n_w << 0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001;
    n_bw << 0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001;

    p_ci << 0, 0, 0.08;
    q_ci.w() = 1.0;
    q_ci.x() = 0.0;
    q_ci.y() = 0.0;
    q_ci.z() = 0.0;
    Rk << 0.01,0,0,0,0,0, 0,0.01,0,0,0,0, 0,0,0.01,0,0,0, 0,0,0,0.01,0,0, 0,0,0,0,0.01,0, 0,0,0,0,0,0.01;
    Xkpre.setZero();
    Kk.setZero();
    Pkpre << 0.01,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0.01,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0.01,0,0,0,0,0,0,0,0,0,0,0,0,
             0,0,0,0.0001,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0.0001,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0.0001,0,0,0,0,0,0,0,0,0,
             0,0,0,0,0,0,0.01,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0.01,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0.01,0,0,0,0,0,0,
             0,0,0,0,0,0,0,0,0,0.01,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0.01,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0.01,0,0,0,
             0,0,0,0,0,0,0,0,0,0,0,0,0.00001,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0.00001,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.00001;

    imu_sub = nhimu.subscribe<sensor_msgs::Imu>("/imu/data", 50, &Imuposeinf::imuCallback, this);
    orb_sub = nhorb.subscribe<nav_msgs::Odometry>("/orbslam/path", 2, &Imuposeinf::orbCallback, this);
    imupos_pub = nhimu.advertise<nav_msgs::Odometry>("imu/path", 50);
    fusion_pub = nhorb.advertise<nav_msgs::Odometry>("fusion/path" ,50);

}

Eigen::Matrix<double,3,3> Imuposeinf::Qua2Dcm(const Eigen::Quaternion<double> q)
{
    Eigen::Matrix<double,3,3> dcm;
    dcm(0,0) = q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z();
    dcm(0,1) = 2*(q.x()*q.y()-q.w()*q.z());
    dcm(0,2) = 2*(q.x()*q.z()+q.w()*q.y());
    dcm(1,0) = 2*(q.x()*q.y()+q.w()*q.z());
    dcm(1,1) = q.w()*q.w()-q.x()*q.x()+q.y()*q.y()-q.z()*q.z();
    dcm(1,2) = 2*(q.y()*q.z()-q.w()*q.x());
    dcm(2,0) = 2*(q.x()*q.z()-q.w()*q.y());
    dcm(2,1) = 2*(q.y()*q.z()+q.w()*q.x());
    dcm(2,2) = q.w()*q.w()-q.x()*q.x()-q.y()*q.y()+q.z()*q.z();
    return dcm;
}

Eigen::Quaternion<double> Imuposeinf::Dcm2Qua(const Eigen::Matrix<double,3,3> dcm)
{
    Eigen::Quaternion<double> q;
    q.w() = sqrt(fabs(1 + dcm(0,0) + dcm(1,1) + dcm(2,2)))/2;
    q.x() = (dcm(2,1) - dcm(1,2))/(4*q.w());
    q.y() = (dcm(0,2) - dcm(2,0))/(4*q.w());
    q.z() = (dcm(1,0) - dcm(0,1))/(4*q.w());
    return q;
}

void Imuposeinf::imuCallback(const sensor_msgs::Imu::ConstPtr& imuIn)
{
      mImuInf_cur.mTime=imuIn->header.stamp.toSec();

      mImuInf_cur.mImu_w << imuIn->angular_velocity.x, imuIn->angular_velocity.y, imuIn->angular_velocity.z;
      mImuInf_cur.mImu_a << imuIn->linear_acceleration.x, imuIn->linear_acceleration.y, imuIn->linear_acceleration.z;
      mImuInf_cur.mImu_ori.w() = imuIn->orientation.w;
      mImuInf_cur.mImu_ori.x() = imuIn->orientation.x;
      mImuInf_cur.mImu_ori.y() = imuIn->orientation.y;
      mImuInf_cur.mImu_ori.z() = imuIn->orientation.z;

      static Eigen::Matrix<double, 3, 1> last_a = Eigen::Matrix<double, 3, 1>(0, 0, 0);
      if (mImuInf_cur.mImu_a.norm() > 50)
      {
          mImuInf_cur.mImu_a = last_a;
      }
      else
      {
          last_a = mImuInf_cur.mImu_a;
      }

      delta_T = mImuInf_cur.mTime - mImuInf_pre.mTime;

      if(mIMUOriCount == false)//初始化为false
      {
          Cgni = mImuInf_cur.mImu_ori.toRotationMatrix();//将四元数转换为旋转矩阵
          gn = Cgni * gn;//gn为重力加速度，得到重力加速度在IMU体坐标系的分量
          //q_pre = mImu_ori;
          delta_T = 0.005;
          mIMUOriCount = true;
      }

      //ROS_INFO("delta_Time is %f ", delta_Time);
      deltatheta = (mImuInf_cur.mImu_w - bw)*delta_T;
      deltatheta_mo = sqrt(deltatheta(0)*deltatheta(0)+deltatheta(1)*deltatheta(1)+deltatheta(2)*deltatheta(2));

      deltav = (mImuInf_cur.mImu_a - ba)*delta_T;//速度增量
      deltav_mo = sqrt(deltav(0)*deltav(0) + deltav(1)*deltav(1) + deltav(2)*deltav(2));

      //姿态解算
      q_bpart = (sin(deltatheta_mo/2)/deltatheta_mo)*deltatheta;
      q_bpre.w() = cos(deltatheta_mo/2);
      q_bpre.x() = q_bpart(0);
      q_bpre.y() = q_bpart(1);
      q_bpre.z() = q_bpart(2);

      q_now = q_pre * q_bpre;
      q_now = q_now.normalized();//归一化

      tf::Quaternion orientation;    //quaternion为四元数
      orientation.setW(q_now.w());
      orientation.setX(q_now.x());
      orientation.setY(q_now.y());
      orientation.setZ(q_now.z());
      tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);//get the matrix represented as roll pitch and yaw about fixed axes XYZ.
      //相对imu体坐标系

      d_Vgn = gn * delta_T;
      d_Vrot = 0.5 * skew(deltatheta) * deltav;//d_Vrot过渡量
      //姿态解算
      V_now = V_pre + d_Vgn + Cbn_pre * (0.5*deltav + d_Vrot);
      //位置解算
      P_now = P_pre + (V_now + V_pre)/2*delta_T;

      //更新
      q_pre = q_now;
      V_pre = V_now;
      P_pre = P_now;
      Cbn_pre = q_pre.toRotationMatrix();

      mImuInf_cur.mPos = P_pre;
      mImuInf_cur.mQua = q_pre;
      mImuInf_pre = mImuInf_cur;
      
      imuPath.header.stamp = imuIn->header.stamp;
      imuPath.header.frame_id = "/calculateIMU";
      imuPath.pose.pose.orientation.x = q_pre.x();
      imuPath.pose.pose.orientation.y = q_pre.y();
      imuPath.pose.pose.orientation.z = q_pre.z();
      imuPath.pose.pose.orientation.w = q_pre.w();
      imuPath.pose.pose.position.x = P_pre.x();
      imuPath.pose.pose.position.y = P_pre.y();
      imuPath.pose.pose.position.z = P_pre.z();
      imupos_pub.publish(imuPath);
}

void Imuposeinf::orbCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    curorb_T = msg->header.stamp.toSec();
    p_orb << msg->pose.pose.position.z, msg->pose.pose.position.x, msg->pose.pose.position.y;
    q_orb.w() = msg->pose.pose.orientation.w;
    q_orb.x() = msg->pose.pose.orientation.z;
    q_orb.y() = msg->pose.pose.orientation.x;
    q_orb.z() = msg->pose.pose.orientation.y;

    dorb_T = curorb_T - preorb_T;
    if(morbflag == false)
    {
        dorb_T = 0.03;
        morbflag =true;
        P_now << 0,0,0;
    }

    Accxx = skew(deltav/delta_T);
    Angxx = skew(deltatheta/delta_T);
    Eigen::Matrix<double,3,3> I3;
    Eigen::Matrix<double,15,15> I15;
    Eigen::Matrix<double,3,3> A,B,C,D,E,F,G;
    I3.setIdentity();
    I15.setIdentity();

    A = -0.5 * Cbn_pre * Accxx * dorb_T * dorb_T;
    B = I3 - Angxx * dorb_T + 0.5 * Angxx * Angxx * dorb_T * dorb_T;
    C = -Cbn_pre * Accxx * dorb_T + 0.5 * Cbn_pre * Accxx * Angxx * dorb_T * dorb_T;
    D = -0.5 * Cbn_pre * dorb_T * dorb_T;
    E = -Cbn_pre * dorb_T;
    F = -I3 * dorb_T + 0.5 * Angxx * dorb_T * dorb_T;
    G = 0.5 * Cbn_pre * Accxx * dorb_T * dorb_T;

    F_fai.setIdentity();
    F_fai.block<3, 3> (0, 3) = A;
    F_fai.block<3, 3> (0, 6) = I3 * dorb_T;
    F_fai.block<3, 3> (0, 9) = D;
    F_fai.block<3, 3> (3, 3) = B;
    F_fai.block<3, 3> (3, 12) = F;
    F_fai.block<3, 3> (6, 3) = C;
    F_fai.block<3, 3> (6, 9) = E;
    F_fai.block<3, 3> (6, 12) = G;

    Q_d.setZero();

    Eigen::Matrix<double,3,3> Cnb_pre = Cbn_pre.transpose();
    double tt = dorb_T * dorb_T;
    double ttt = dorb_T * dorb_T * dorb_T;
    double tttt = dorb_T * dorb_T * dorb_T * dorb_T;

    Q_d.block<3, 3> (0, 0) = n_a * ttt/3;
    Q_d.block<3, 3> (0, 3) = -Cbn_pre * Accxx * n_w * ttt/6;
    Q_d.block<3, 3> (0, 6) = n_a * tt/2;
    Q_d.block<3, 3> (0, 9) = -Cbn_pre * n_ba * ttt/6;
    Q_d.block<3, 3> (3, 0) = Accxx * Cnb_pre * n_w * ttt/6 + Cnb_pre * n_bw * ttt/3;
    Q_d.block<3, 3> (3, 3) = n_w * dorb_T + n_bw * ttt/3;
    Q_d.block<3, 3> (3, 6) = Accxx * Cnb_pre * n_w * tt/2 - Angxx * Accxx * Cnb_pre * n_w * ttt/6;
    Q_d.block<3, 3> (3, 12) = -n_bw * tt/2 + Angxx * n_bw * ttt/6;
    Q_d.block<3, 3> (6, 0) = n_a * tt/2;
    Q_d.block<3, 3> (6, 3) = -Cbn_pre * Accxx * n_bw * tttt/8;
    Q_d.block<3, 3> (6, 6) = n_a * dorb_T + n_ba * ttt/3;
    Q_d.block<3, 3> (6, 9) = -n_ba * Cbn_pre * tt/2;
    Q_d.block<3, 3> (6, 12) = Cbn_pre * Accxx * n_bw * ttt/6;
    Q_d.block<3, 3> (9, 0) = -Cnb_pre * n_ba * tt/6;
    Q_d.block<3, 3> (9, 6) = -Cnb_pre * n_ba * tt/2;
    Q_d.block<3, 3> (9, 9) = n_ba * dorb_T;
    Q_d.block<3, 3> (12, 0) = -Cnb_pre * n_bw * tt/2;
    Q_d.block<3, 3> (12, 3) = -n_bw * tt/2 - Angxx * n_bw * ttt/6;
    Q_d.block<3, 3> (12, 6) = -Accxx * Cnb_pre * n_bw * ttt/6;
    Q_d.block<3, 3> (12, 12) = n_bw * dorb_T;

    // Z_p = P_w_c = P_w_i + C_w_i * P_i_c
    // Z_q : https://github.com/ethz-asl/ethzasl_sensor_fusion/blob/master/ssf_updates/src/pose_sensor.cpp
    p_error = p_orb - P_now - Cnb_pre * p_ci;
    q_e = (q_now * q_ci).conjugate() * q_orb;
    q_error = q_e.vec()/q_e.w()*2;
    z_k << p_error, q_error;

    Pcixx = skew(p_ci);
    H_k.setZero();
    H_k.block<3, 3> (0, 0) = I3;
    H_k.block<3, 3> (0, 3) = -Cbn_pre * Pcixx;
    H_k.block<3, 3> (3, 3) = I3;

    Xkpre = F_fai * Xkpre;
    Pkpre = F_fai * Pkpre * F_fai.transpose() + Q_d;
    Kk = Pkpre * H_k.transpose() * (H_k * Pkpre * H_k.transpose() + Rk).inverse();
    Xkpre = Xkpre + Kk * (z_k - H_k * Xkpre);
    Pkpre = (I15 - Kk * H_k) * Pkpre * (I15 - Kk * H_k).transpose() + Kk * Rk * Kk.transpose();

    fai_ver << 0, -Xkpre(5), Xkpre(4), Xkpre(5), 0, -Xkpre(3), -Xkpre(4), Xkpre(3), 0;

    Cbn_now = Cbn_pre * (I3 + fai_ver);
    P_now = P_now + Xkpre.block<3, 1> (0, 0);
    V_now = V_now + Xkpre.block<3, 1> (6, 0);
    ba = ba + Xkpre.block<3, 1> (9, 0);
    bw = bw + Xkpre.block<3, 1> (12, 0);
    Xkpre.setZero();

    Cbn_pre = Cbn_now;
    q_now = Dcm2Qua(Cbn_pre);
    q_now = q_now.normalized();

    q_pre = q_now;
    V_pre = V_now;
    P_pre = P_now;

    mImuInf_cur.mPos = P_pre;
    mImuInf_cur.mQua = q_pre;
    mImuInf_pre = mImuInf_cur;

    fusionPath.header.stamp = msg->header.stamp;
    fusionPath.header.frame_id = "/calculateFusion";
    fusionPath.pose.pose.orientation.x = q_pre.x();
    fusionPath.pose.pose.orientation.y = q_pre.y();
    fusionPath.pose.pose.orientation.z = q_pre.z();
    fusionPath.pose.pose.orientation.w = q_pre.w();
    fusionPath.pose.pose.position.x = P_pre.x();
    fusionPath.pose.pose.position.y = P_pre.y();
    fusionPath.pose.pose.position.z = P_pre.z();
    fusion_pub.publish(fusionPath);

    preorb_T = curorb_T;
}




