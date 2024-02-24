#ifndef FASTER_LIO_IMU_PROCESSING_H
#define FASTER_LIO_IMU_PROCESSING_H

#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <deque>
#include <fstream>

#include "common_lib.h"
#include "so3_math.h"
#include "use-ikfom.hpp"
#include "utils.h"

namespace faster_lio {

// #define cost
// #define debug

constexpr int MAX_INI_COUNT = 20;

bool time_list(const PointType &x, const PointType &y) { return (x.curvature < y.curvature); };

class LidarBag {
    public:
     PointCloudType::Ptr point_cloud;
     double lidar_beg_time;
     double lidar_end_time;
};

/// IMU Process and undistortion
class ImuProcess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    void Reset();
    bool NeedInit();
    void SetExtrinsic(const common::V3D &transl, const common::M3D &rot);
    void SetGyrCov(const common::V3D &scaler);
    void SetAccCov(const common::V3D &scaler);
    void SetGyrBiasCov(const common::V3D &b_g);
    void SetAccBiasCov(const common::V3D &b_a);
    void ImuUpdate(esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state);
    void Process(esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                 PointCloudType::Ptr pcl_un_);

    std::ofstream fout_imu_;
    Eigen::Matrix<double, 12, 12> Q_;
    common::V3D cov_acc_;
    common::V3D cov_gyr_;
    common::V3D cov_acc_scale_;
    common::V3D cov_gyr_scale_;
    common::V3D cov_bias_gyr_;
    common::V3D cov_bias_acc_;
    std::deque<LidarBag> lidar_buffer_;
    std::deque<sensor_msgs::ImuConstPtr> front_imu_;
    std::deque<sensor_msgs::ImuConstPtr> back_imu_;
    sensor_msgs::ImuConstPtr last_imu_;
    double last_timestamp_ = 0;
    bool new_odom_ = false;
    bool new_cloud_ = false;

   private:
    void IMUInit(esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
    void UndistortPcl(esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                      PointCloudType &pcl_out);

    PointCloudType::Ptr cur_pcl_un_;
    std::vector<common::Pose6D> IMUpose_;
    std::vector<common::M3D> v_rot_pcl_;
    common::M3D Lidar_R_wrt_IMU_;
    common::V3D Lidar_T_wrt_IMU_;
    common::V3D mean_acc_;
    common::V3D mean_gyr_;
    common::V3D angvel_last_;
    common::V3D acc_s_last_;
    double last_lidar_end_time_ = 0;
    double start_time_;
    int init_iter_num_ = 1;
    bool b_first_frame_ = true;
    bool imu_need_init_ = true;
};

ImuProcess::ImuProcess() : b_first_frame_(true), imu_need_init_(true) {
    init_iter_num_ = 1;
    Q_ = process_noise_cov();
    cov_acc_ = common::V3D(0.1, 0.1, 0.1);
    cov_gyr_ = common::V3D(0.1, 0.1, 0.1);
    cov_bias_gyr_ = common::V3D(0.0001, 0.0001, 0.0001);
    cov_bias_acc_ = common::V3D(0.0001, 0.0001, 0.0001);
    mean_acc_ = common::V3D(0, 0, -1.0);
    mean_gyr_ = common::V3D(0, 0, 0);
    angvel_last_ = common::Zero3d;
    Lidar_T_wrt_IMU_ = common::Zero3d;
    Lidar_R_wrt_IMU_ = common::Eye3d;
    lidar_buffer_.clear();
    front_imu_.clear();
    back_imu_.clear();
    IMUpose_.clear();
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
    mean_acc_ = common::V3D(0, 0, -1.0);
    mean_gyr_ = common::V3D(0, 0, 0);
    angvel_last_ = common::Zero3d;
    imu_need_init_ = true;
    init_iter_num_ = 1;
    cur_pcl_un_.reset(new PointCloudType());
}

void ImuProcess::SetExtrinsic(const common::V3D &transl, const common::M3D &rot) {
    Lidar_T_wrt_IMU_ = transl;
    Lidar_R_wrt_IMU_ = rot;
}

bool ImuProcess::NeedInit() { return imu_need_init_; }

void ImuProcess::SetGyrCov(const common::V3D &scaler) { cov_gyr_scale_ = scaler; }

void ImuProcess::SetAccCov(const common::V3D &scaler) { cov_acc_scale_ = scaler; }

void ImuProcess::SetGyrBiasCov(const common::V3D &b_g) { cov_bias_gyr_ = b_g; }

void ImuProcess::SetAccBiasCov(const common::V3D &b_a) { cov_bias_acc_ = b_a; }

void ImuProcess::IMUInit(esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                         int &N) {
    
    /** 1. initializing the gravity_, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity_ **/

    common::V3D cur_acc, cur_gyr;

    if (b_first_frame_) {
        Reset();
        N = 1;
        b_first_frame_ = false;
        const auto &imu_acc = back_imu_.front()->linear_acceleration;
        const auto &gyr_acc = back_imu_.front()->angular_velocity;
        mean_acc_ << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr_ << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    }

    for (const auto &imu : back_imu_) {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        mean_acc_ += (cur_acc - mean_acc_) / N;
        mean_gyr_ += (cur_gyr - mean_gyr_) / N;

        cov_acc_ =
            cov_acc_ * (N - 1.0) / N + (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
        cov_gyr_ =
            cov_gyr_ * (N - 1.0) / N + (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);

        N++;
    }

    front_imu_ = back_imu_;
    back_imu_.clear();

    state_ikfom init_state = kf_state.get_x();
    init_state.grav = S2(-mean_acc_ / mean_acc_.norm() * common::G_m_s2);

    init_state.bg = mean_gyr_;
    init_state.offset_T_L_I = Lidar_T_wrt_IMU_;
    init_state.offset_R_L_I = Lidar_R_wrt_IMU_;
    kf_state.change_x(init_state);

    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
    init_P.setIdentity();
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    kf_state.change_P(init_P);
}

void ImuProcess::UndistortPcl(esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                              PointCloudType &pcl_out) {

#ifdef cost
    double t = ros::Time::now().toNSec();
#endif

    // while (!lidar_buffer_.empty() && last_imu_->header.stamp.toSec() - lidar_buffer_.front().lidar_beg_time > 0.1)
    //     lidar_buffer_.pop_front();

    if (lidar_buffer_.empty() || lidar_buffer_.front().lidar_end_time > last_timestamp_)
        return;

    double pcl_beg_time = lidar_buffer_.front().lidar_beg_time;
    double pcl_end_time = lidar_buffer_.front().lidar_end_time;
    pcl_out = *(lidar_buffer_.front().point_cloud);
    lidar_buffer_.pop_front();


    // if (last_imu_->header.stamp.toSec() - pcl_end_time > 0.1)
    //     return;

    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);

#ifdef debug
    std::cout << std::endl << "----------UNDISTORLPCL----------" << std::endl << "UNCUTTED_POSE:" << std::endl; 
    for (auto it = IMUpose_.begin(); it != IMUpose_.end(); it++) {
        auto &&p = *(it);
        common::M3D R = common::MatFromArray(it->rot);
        tf::Matrix3x3 tf_R(R(0,0), R(0,1), R(0,2),
                           R(1,0), R(1,1), R(1,2),
                           R(2,0), R(2,1), R(2,2));
        double roll, pitch, yaw;
        tf_R.getRPY(roll, pitch, yaw);
        std::cout << p.offset_time - start_time_ << ":" << std::endl << common::VecFromArray(it->pos) << std::endl << roll << " " << pitch << " " << yaw << std::endl << "***" << std::endl;
    }
    std::cout << std::endl << "lidar_bag_time:" << pcl_beg_time - start_time_ << std::endl; 
    std::cout << "pcl_end_time:" << pcl_end_time - start_time_ << std::endl; 
#endif

    if (!IMUpose_.empty())
    {
        for (auto it_pose = IMUpose_.rbegin(); it_pose != IMUpose_.rend();)
        {
            if (it_pose->offset_time >= pcl_end_time)
            {
                it_pose++;
                IMUpose_.pop_back();
                continue;
            }
            else
                break;
        }
    }

#ifdef debug
    std::cout << std::endl << "----------UNDISTORLPCL----------" << std::endl << "CUTTED_POSE:" << std::endl; 
    for (auto it = IMUpose_.begin(); it != IMUpose_.end(); it++) {
        auto &&p = *(it);
        std::cout << p.offset_time - start_time_ << " ";
    }
    std::cout << std::endl << "lidar_bag_time:" << pcl_beg_time - start_time_ << std::endl; 
    std::cout << "pcl_end_time:" << pcl_end_time - start_time_ << std::endl; 

    std::cout << std::endl << "----------BEFORESORT----------" << std::endl << "front_imu:" << std::endl; 
    for (auto it = front_imu_.begin(); it != front_imu_.end(); it++) {
        auto &&p = *(it);
        std::cout << p->header.stamp.toSec() - start_time_ << " ";
    }
    std::cout << std::endl << "back_imu:" << std::endl;
    for (auto it = back_imu_.begin(); it != back_imu_.end(); it++) {
        auto &&p = *(it);
        std::cout << p->header.stamp.toSec() - start_time_ << " ";
    }
    std::cout << std::endl << "last_odom_time_:" << last_imu_->header.stamp.toSec() - start_time_ << std::endl; 
#endif

    /*** forward propagation at each imu_ point ***/
    common::V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    common::M3D R_imu;

    double dt = 0;

    input_ikfom in;
    sensor_msgs::Imu::Ptr msg;
    if (last_imu_->header.stamp.toSec() > pcl_end_time)
    {
        while (!front_imu_.empty()) {
            auto &&p_imu = front_imu_.back();

            angvel_avr << 0.5 * (last_imu_->angular_velocity.x + p_imu->angular_velocity.x),
                0.5 * (last_imu_->angular_velocity.y + p_imu->angular_velocity.y),
                0.5 * (last_imu_->angular_velocity.z + p_imu->angular_velocity.z);
            acc_avr << 0.5 * (last_imu_->linear_acceleration.x + p_imu->linear_acceleration.x),
                0.5 * (last_imu_->linear_acceleration.y + p_imu->linear_acceleration.y),
                0.5 * (last_imu_->linear_acceleration.z + p_imu->linear_acceleration.z);

            acc_avr = acc_avr * common::G_m_s2 / mean_acc_.norm();  // - state_inout.ba;

            in.acc = acc_avr;
            in.gyro = angvel_avr;
            Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
            Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
            Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
            Q_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;

            if (p_imu->header.stamp.toSec() < pcl_end_time) {
                msg = sensor_msgs::Imu::Ptr(new sensor_msgs::Imu(*p_imu));
                msg->header.stamp = ros::Time().fromSec(pcl_end_time);
                back_imu_.push_front(last_imu_);
                front_imu_.clear();
                break;
            }

            dt = p_imu->header.stamp.toSec() - last_imu_->header.stamp.toSec();
            kf_state.predict(dt, Q_, in);
            
#ifdef debug
            state_ikfom imu_state = kf_state.get_x();
            tf::Quaternion q(imu_state.rot.coeffs()[0], imu_state.rot.coeffs()[1], imu_state.rot.coeffs()[2], imu_state.rot.coeffs()[3]);
            double roll, pitch, yaw;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            std::cout << "******" << std::endl << p_imu->header.stamp.toSec() - start_time_ << ":" << std::endl << kf_state.get_x().pos << std::endl  << roll << " " << pitch << " " << yaw << std::endl << "********" << std::endl;
    std::cout << std::endl << "----------UNDISTORLPCL----------" << std::endl << "DT:" << dt << std::endl; 
#endif

            back_imu_.push_front(last_imu_);
            last_imu_ = p_imu;
            front_imu_.pop_back();

        }
    }
    else
    {
        while (!back_imu_.empty())
        {
            auto &&p_imu = back_imu_.front();

            angvel_avr << 0.5 * (last_imu_->angular_velocity.x + p_imu->angular_velocity.x),
                0.5 * (last_imu_->angular_velocity.y + p_imu->angular_velocity.y),
                0.5 * (last_imu_->angular_velocity.z + p_imu->angular_velocity.z);
            acc_avr << 0.5 * (last_imu_->linear_acceleration.x + p_imu->linear_acceleration.x),
                0.5 * (last_imu_->linear_acceleration.y + p_imu->linear_acceleration.y),
                0.5 * (last_imu_->linear_acceleration.z + p_imu->linear_acceleration.z);

            acc_avr = acc_avr * common::G_m_s2 / mean_acc_.norm();  // - state_inout.ba;

            in.acc = acc_avr;
            in.gyro = angvel_avr;
            Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
            Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
            Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
            Q_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;

            if (p_imu->header.stamp.toSec() > pcl_end_time){
                msg = sensor_msgs::Imu::Ptr(new sensor_msgs::Imu(*last_imu_));
                msg->header.stamp = ros::Time().fromSec(pcl_end_time);
                front_imu_.clear();
                break;
            }

            dt = p_imu->header.stamp.toSec() - last_imu_->header.stamp.toSec();
            kf_state.predict(dt, Q_, in);

#ifdef debug
    std::cout << std::endl << "----------UNDISTORLPCL----------" << std::endl << "DT:" << dt << std::endl; 
#endif

            front_imu_.push_back(last_imu_);
            last_imu_ = p_imu;
            back_imu_.pop_front();

            state_ikfom imu_state = kf_state.get_x();
            angvel_last_ = angvel_avr - imu_state.bg;
            acc_s_last_ = imu_state.rot * (acc_avr - imu_state.ba);
            for (int i = 0; i < 3; i++) {
                acc_s_last_[i] += imu_state.grav[i];
            }

            double &&time = last_imu_->header.stamp.toSec();
            IMUpose_.emplace_back(common::set_pose6d(time, acc_s_last_, angvel_last_, imu_state.vel, imu_state.pos,
                                                    imu_state.rot.toRotationMatrix()));

        }
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    dt = pcl_end_time - last_imu_->header.stamp.toSec();
    kf_state.predict(dt, Q_, in);

#ifdef debug
    std::cout << std::endl << "----------UNDISTORLPCL----------" << std::endl << "DT:" << dt << std::endl; 
#endif

    last_imu_ = msg;

    state_ikfom imu_state = kf_state.get_x();

#ifdef debug
    std::cout << std::endl << "----------AFTERSORT----------" << std::endl << "front_imu:" << std::endl; 
    for (auto it = front_imu_.begin(); it != front_imu_.end(); it++) {
        auto &&p = *(it);
        std::cout << p->header.stamp.toSec() - start_time_ << " ";
    }
    std::cout << std::endl << "back_imu:" << std::endl;
    for (auto it = back_imu_.begin(); it != back_imu_.end(); it++) {
        auto &&p = *(it);
        std::cout << p->header.stamp.toSec() - start_time_ << " ";
    }
    std::cout << std::endl << "last_odom_time_:" << last_imu_->header.stamp.toSec() - start_time_ << std::endl; 

    std::cout << std::endl << "----------UNDISTORLPCL----------" << std::endl << "AFTER_POSE:" << std::endl; 
    for (auto it = IMUpose_.begin(); it != IMUpose_.end(); it++) {
        auto &&p = *(it);
        common::M3D R = common::MatFromArray(it->rot);
        tf::Matrix3x3 tf_R(R(0,0), R(0,1), R(0,2),
                             R(1,0), R(1,1), R(1,2),
                             R(2,0), R(2,1), R(2,2));
        double roll, pitch, yaw;
        tf_R.getRPY(roll, pitch, yaw);
        std::cout << p.offset_time - start_time_ << ":" << std::endl << common::VecFromArray(it->pos) << std::endl << roll << " " << pitch << " " << yaw << std::endl << "***" << std::endl;
    }
    std::cout << std::endl << "lidar_bag_time:" << pcl_beg_time - start_time_ << std::endl; 
    std::cout << "pcl_end_time:" << pcl_end_time - start_time_ << std::endl; 
#endif

    /*** undistort each lidar point (backward propagation) ***/
    if (pcl_out.points.empty() || IMUpose_.empty()) {
        return;
    }
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = IMUpose_.end() - 1; it_kp != IMUpose_.begin(); it_kp--) {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu = common::MatFromArray(head->rot);
        vel_imu = common::VecFromArray(head->vel);
        pos_imu = common::VecFromArray(head->pos);
        acc_imu = common::VecFromArray(tail->acc);
        angvel_avr = common::VecFromArray(tail->gyr);

        for (; it_pcl->curvature / double(1000) > head->offset_time - pcl_beg_time; it_pcl--) {
            dt = it_pcl->curvature / double(1000) - (head->offset_time - pcl_beg_time);

            /* Transform to the 'end' frame, using only the rotation
             * Note: Compensation direction is INVERSE of Frame's moving direction
             * So if we want to compensate a point at timestamp-i to the frame-e
             * p_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
            common::M3D R_i(R_imu * Exp(angvel_avr, dt));

            common::V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            common::V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
            common::V3D p_compensate =
                imu_state.offset_R_L_I.conjugate() *
                (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
                 imu_state.offset_T_L_I);  // not accurate!

            // save Undistorted points and their rotation
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);

            if (it_pcl == pcl_out.points.begin()) {
                break;
            }
        }
    }

    IMUpose_.clear();
    /* save the poses at each IMU measurements */
    angvel_last_ = angvel_avr - imu_state.bg;
    acc_s_last_ = imu_state.rot * (acc_avr - imu_state.ba);
    for (int i = 0; i < 3; i++) {
        acc_s_last_[i] += imu_state.grav[i];
    }
    
    double time = pcl_end_time;
    IMUpose_.emplace_back(common::set_pose6d(time, acc_s_last_, angvel_last_, imu_state.vel, imu_state.pos,
                                                 imu_state.rot.toRotationMatrix()));

#ifdef cost
    std::cout << "UndistortPcl Cost:" << 1000 * (ros::Time::now().toSec() - t) << "ms" << std::endl;;
#endif

    new_cloud_ = true;
    return;
}

void ImuProcess::ImuUpdate(esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state)
{

#ifdef cost
    double t = ros::Time::now().toSec();
#endif

    if (back_imu_.empty())
        return;

#ifdef debug
    std::cout << std::endl << "----------BEFOREUPDATE----------" << std::endl << "front_imu:" << std::endl; 
    for (auto it = front_imu_.begin(); it != front_imu_.end(); it++) {
        auto &&p = *(it);
        std::cout << p->header.stamp.toSec() - start_time_ << " ";
    }
    std::cout << std::endl << "back_imu:" << std::endl;
    for (auto it = back_imu_.begin(); it != back_imu_.end(); it++) {
        auto &&p = *(it);
        std::cout << p->header.stamp.toSec() - start_time_ << " ";
    }
    std::cout << std::endl << "last_odom_time_:" << last_imu_->header.stamp.toSec() - start_time_ << std::endl; 
#endif

    /*** forward propagation at each imu ***/
    common::V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    common::M3D R_imu;

    double dt = 0;

    input_ikfom in;
    while (!back_imu_.empty()) {

        auto&& p_imu = back_imu_.front();

        angvel_avr << 0.5 * (p_imu->angular_velocity.x + p_imu->angular_velocity.x),
            0.5 * (p_imu->angular_velocity.y + p_imu->angular_velocity.y),
            0.5 * (p_imu->angular_velocity.z + p_imu->angular_velocity.z);
        acc_avr << 0.5 * (p_imu->linear_acceleration.x + p_imu->linear_acceleration.x),
            0.5 * (p_imu->linear_acceleration.y + p_imu->linear_acceleration.y),
            0.5 * (p_imu->linear_acceleration.z + p_imu->linear_acceleration.z);

        acc_avr = acc_avr * common::G_m_s2 / mean_acc_.norm();  // - state_inout.ba;

        in.acc = acc_avr;
        in.gyro = angvel_avr;
        Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
        Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
        Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
        Q_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;

        dt = p_imu->header.stamp.toSec() - last_imu_->header.stamp.toSec();
        kf_state.predict(dt, Q_, in);

        /* save the poses at each IMU measurements */
        state_ikfom imu_state = kf_state.get_x();
        angvel_last_ = angvel_avr - imu_state.bg;
        acc_s_last_ = imu_state.rot * (acc_avr - imu_state.ba);
        for (int i = 0; i < 3; i++) {
            acc_s_last_[i] += imu_state.grav[i];
        }

        double &&time = p_imu->header.stamp.toSec();
        IMUpose_.emplace_back(common::set_pose6d(time, acc_s_last_, angvel_last_, imu_state.vel, imu_state.pos,
                                                 imu_state.rot.toRotationMatrix()));

        front_imu_.push_back(last_imu_);
        last_imu_ = back_imu_.front();
        back_imu_.pop_front();

        if (!lidar_buffer_.empty() && lidar_buffer_.front().lidar_end_time < back_imu_.back()->header.stamp.toSec())
            return;
    }

#ifdef debug
    std::cout << std::endl << "----------AFTERUPDATE----------" << std::endl << "front_imu:" << std::endl; 
    for (auto it = front_imu_.begin(); it != front_imu_.end(); it++) {
        auto &&p = *(it);
        std::cout << p->header.stamp.toSec() - start_time_ << " ";
    }
    std::cout << std::endl << "back_imu:" << std::endl;
    for (auto it = back_imu_.begin(); it != back_imu_.end(); it++) {
        auto &&p = *(it);
        std::cout << p->header.stamp.toSec() - start_time_ << " ";
    }
    std::cout << std::endl << "last_odom_time_:" << last_imu_->header.stamp.toSec() - start_time_ << std::endl; 
#endif

    new_odom_ = true;

#ifdef cost
    std::cout << "ImuUpdate Cost:" << 1000 * (ros::Time::now().toSec() - t) << "ms" << std::endl;
#endif
    return;

}

void ImuProcess::Process(esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                         PointCloudType::Ptr cur_pcl_un_) {

    if (imu_need_init_) {

        if (back_imu_.size() < 30) {
            return;
        }
        
        /// The very first lidar frame
        IMUInit(kf_state, init_iter_num_);

        imu_need_init_ = true;
        last_imu_ = front_imu_.back();
        start_time_ = last_imu_->header.stamp.toSec();

        if (init_iter_num_ > MAX_INI_COUNT) {
            lidar_buffer_.clear();
            cov_acc_ *= pow(common::G_m_s2 / mean_acc_.norm(), 2);
            imu_need_init_ = false;

            cov_acc_ = cov_acc_scale_;
            cov_gyr_ = cov_gyr_scale_;
            LOG(INFO) << "IMU Initial Done";
            fout_imu_.open(common::DEBUG_FILE_DIR("imu_.txt"), std::ios::out);

        }

        return;
    }

    Timer::Evaluate([&, this]() { UndistortPcl(kf_state, *cur_pcl_un_); }, "Undistort Pcl");
    if (!new_cloud_)
        Timer::Evaluate([&, this]() { ImuUpdate(kf_state); }, "Imu Update");

}

}  // namespace faster_lio

#endif
