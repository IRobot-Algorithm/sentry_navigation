#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <execution>
#include <fstream>
#include <iomanip>

#include "laser_mapping.h"
#include "utils.h"

namespace faster_lio {

bool LaserMapping::InitROS(ros::NodeHandle &nh) {
    while (!LoadParams(nh));
    SubAndPubToROS(nh);

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    std::vector<double> epsi(23, 0.001);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS, epsi.data());

    return true;
}

bool LaserMapping::InitWithoutROS(const std::string &config_yaml) {
    LOG(INFO) << "init laser mapping from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    std::vector<double> epsi(23, 0.001);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS, epsi.data());

    if (std::is_same<IVoxType, IVox<3, IVoxNodeType::PHC, pcl::PointXYZI>>::value == true) {
        LOG(INFO) << "using phc ivox";
    } else if (std::is_same<IVoxType, IVox<3, IVoxNodeType::DEFAULT, pcl::PointXYZI>>::value == true) {
        LOG(INFO) << "using default ivox";
    }

    return true;
}

bool LaserMapping::LoadParams(ros::NodeHandle &nh) {
    // get params from param server
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;
    std::vector<double> extrinT_IMU_BOT{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR_IMU_BOT{9, 0.0};  // lidar-imu rotation

    nh.param<bool>("icp/use_icp", use_icp_, false);
    nh.param<bool>("path_save_en", path_save_en_, true);
    nh.param<bool>("publish/path_publish_en", path_pub_en_, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en_, true);
    nh.param<bool>("publish/dense_publish_en", dense_pub_en_, false);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en_, true);
    nh.param<bool>("publish/scan_effect_pub_en", scan_effect_pub_en_, false);

    nh.param<int>("max_iteration", options::NUM_MAX_ITERATIONS, 4);
    nh.param<float>("esti_plane_threshold", options::ESTI_PLANE_THRESHOLD, 0.1);
    nh.param<std::string>("map_file_path", map_file_path_, "");
    nh.param<bool>("common/time_sync_en", time_sync_en_, false);
    nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("filter_size_map", filter_size_map_min_, 0.0);
    nh.param<double>("cube_side_length", cube_len_, 200);
    nh.param<float>("mapping/det_range", det_range_, 300.f);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh.param<double>("preprocess/blind", preprocess_->Blind(), 0.01);
    nh.param<float>("preprocess/time_scale", preprocess_->TimeScale(), 1e-3);
    nh.param<int>("preprocess/lidar_type", lidar_type, 1);
    nh.param<int>("preprocess/scan_line", preprocess_->NumScans(), 16);
    nh.param<int>("point_filter_num", preprocess_->PointFilterNum(), 2);
    nh.param<bool>("feature_extract_enable", preprocess_->FeatureEnabled(), false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log_, true);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en_, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en_, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval_, -1);
    nh.param<std::vector<double>>("mapping/extrinsic_T", extrinT_, std::vector<double>());
    nh.param<std::vector<double>>("mapping/extrinsic_R", extrinR_, std::vector<double>());
    nh.param<std::vector<double>>("irobot/extrinsic_T", extrinT_IMU_BOT, std::vector<double>());
    nh.param<std::vector<double>>("irobot/extrinsic_R", extrinR_IMU_BOT, std::vector<double>());

    nh.param<float>("ivox_grid_resolution", ivox_options_.resolution_, 0.2);
    nh.param<int>("ivox_nearby_type", ivox_nearby_type, 18);

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    if (use_icp_)
    {
        init_localization_ = true;
    }

    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = "odom";

    voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    // 初始变换矩阵
    IMU_T_wrt_BOT_ = common::VecFromArray<double>(extrinT_IMU_BOT);
    IMU_R_wrt_BOT_ = common::MatFromArray<double>(extrinR_IMU_BOT);
    BOT_T_wrt_IMU_ = -IMU_T_wrt_BOT_;
    BOT_R_wrt_IMU_ = IMU_R_wrt_BOT_.inverse();

    // icp迭代结果
    icp_T_wrt_.setZero();
    icp_R_wrt_.setIdentity();
    
    // 点云最终转换矩阵
    T_wrt_ = BOT_T_wrt_IMU_;
    R_wrt_ = BOT_R_wrt_IMU_;
    R_wrt_inv_ = IMU_R_wrt_BOT_;

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    return true;
}

bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    // get params from yaml
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        path_pub_en_ = yaml["publish"]["path_publish_en"].as<bool>();
        scan_pub_en_ = yaml["publish"]["scan_publish_en"].as<bool>();
        dense_pub_en_ = yaml["publish"]["dense_publish_en"].as<bool>();
        scan_body_pub_en_ = yaml["publish"]["scan_bodyframe_pub_en"].as<bool>();
        scan_effect_pub_en_ = yaml["publish"]["scan_effect_pub_en"].as<bool>();
        path_save_en_ = yaml["path_save_en"].as<bool>();

        options::NUM_MAX_ITERATIONS = yaml["max_iteration"].as<int>();
        options::ESTI_PLANE_THRESHOLD = yaml["esti_plane_threshold"].as<float>();
        time_sync_en_ = yaml["common"]["time_sync_en"].as<bool>();

        filter_size_surf_min = yaml["filter_size_surf"].as<float>();
        filter_size_map_min_ = yaml["filter_size_map"].as<float>();
        cube_len_ = yaml["cube_side_length"].as<int>();
        det_range_ = yaml["mapping"]["det_range"].as<float>();
        gyr_cov = yaml["mapping"]["gyr_cov"].as<float>();
        acc_cov = yaml["mapping"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["mapping"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["mapping"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["preprocess"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["preprocess"]["time_scale"].as<double>();
        lidar_type = yaml["preprocess"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["preprocess"]["scan_line"].as<int>();
        preprocess_->PointFilterNum() = yaml["point_filter_num"].as<int>();
        preprocess_->FeatureEnabled() = yaml["feature_extract_enable"].as<bool>();
        extrinsic_est_en_ = yaml["mapping"]["extrinsic_est_en"].as<bool>();
        pcd_save_en_ = yaml["pcd_save"]["pcd_save_en"].as<bool>();
        pcd_save_interval_ = yaml["pcd_save"]["interval"].as<int>();
        extrinT_ = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();

        ivox_options_.resolution_ = yaml["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["ivox_nearby_type"].as<int>();
    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    run_in_offline_ = true;
    return true;
}

void LaserMapping::SubAndPubToROS(ros::NodeHandle &nh) {
    nh_ = nh;

    // ROS subscribe initialization
    std::string lidar_topic, imu_topic;
    nh.param<std::string>("common/lid_topic", lidar_topic, "/livox/lidar");
    nh.param<std::string>("common/imu_topic", imu_topic, "/livox/imu");

    if (preprocess_->GetLidarType() == LidarType::AVIA) {
        sub_pcl_ = nh.subscribe<livox_ros_driver::CustomMsg>(
            lidar_topic, 200000, [this](const livox_ros_driver::CustomMsg::ConstPtr &msg) { LivoxPCLCallBack(msg); });
    } else {
        sub_pcl_ = nh.subscribe<sensor_msgs::PointCloud2>(
            lidar_topic, 200000, [this](const sensor_msgs::PointCloud2::ConstPtr &msg) { StandardPCLCallBack(msg); });
    }

    sub_imu_ = nh.subscribe<sensor_msgs::Imu>(imu_topic, 200000,
                                              [this](const sensor_msgs::Imu::ConstPtr &msg) { IMUCallBack(msg); });

    sub_color_info_ = nh.subscribe<std_msgs::Bool>("/color_info", 1,
                                              [this](const std_msgs::Bool::ConstPtr &msg) { ColorInfoCallBack(msg); });

    // ROS publisher init
    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = "odom";

    pub_laser_cloud_world_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    pub_laser_cloud_body_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    pub_laser_cloud_effect_world_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_effect_world", 100000);
    pub_odom_aft_mapped_ = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    pub_path_ = nh.advertise<nav_msgs::Path>("/path", 100000);
    pub_vel_ = nh.advertise<geometry_msgs::TwistStamped>("/odom_vel", 100000);
    pub_record_odom_ = nh.advertise<geometry_msgs::PoseStamped>("/record/odom", 1);
    pub_record_imu_ = nh.advertise<sensor_msgs::Imu>("/record/imu", 1);
}

LaserMapping::LaserMapping() {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
}

bool LaserMapping::SyncPackages() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_bag_time_ = time_buffer_.front();

        if (measures_.lidar_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else if (measures_.lidar_->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_bag_time_ + measures_.lidar_->points.back().curvature / double(1000);
            lidar_mean_scantime_ +=
                (measures_.lidar_->points.back().curvature / double(1000) - lidar_mean_scantime_) / scan_num_;
        }

        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = imu_buffer_.front()->header.stamp.toSec();
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time_) break;
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
}

bool LaserMapping::IMUUpdate()
{
    if (imu_buf_.empty())
        return false;

    common::V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    input_ikfom in;
    double dt = 0;

    while (!imu_buf_.empty())
    {
        auto &&p_imu = imu_buf_.front();

        angvel_avr << 0.5 * (last_imu_->angular_velocity.x + p_imu->angular_velocity.x),
                      0.5 * (last_imu_->angular_velocity.y + p_imu->angular_velocity.y),
                      0.5 * (last_imu_->angular_velocity.z + p_imu->angular_velocity.z);
        acc_avr << 0.5 * (last_imu_->linear_acceleration.x + p_imu->linear_acceleration.x),
                   0.5 * (last_imu_->linear_acceleration.y + p_imu->linear_acceleration.y),
                   0.5 * (last_imu_->linear_acceleration.z + p_imu->linear_acceleration.z);

        acc_avr = acc_avr * common::G_m_s2 / p_imu_->mean_acc_.norm();  // - state_inout.ba;

        in.acc = acc_avr;
        in.gyro = angvel_avr;

        dt = p_imu->header.stamp.toSec() - last_imu_->header.stamp.toSec();

        MTK::vectview<const double, 24> f = get_f(state_imu_, in);
        state_imu_.oplus(f, dt);

        last_imu_ = p_imu;
        imu_buf_.pop_front();
    }

    return true;
}

void LaserMapping::Run() {
    if (!color_init_)
    {
        static int color_n = 0;
        color_n++;
        if (color_n > 100)
        {
            color_n = 0;
            
            mtx_buffer_.lock();
            lidar_buffer_.clear();
            time_buffer_.clear();
            imu_buffer_.clear();
            imu_buf_.clear();
            mtx_buffer_.unlock();
            
            LOG(WARNING) << "----> Init Color Failed!";
        }
        return;
    }

    if (use_icp_ && init_localization_)
    {
        if (!relocalization_.IsInit())
            relocalization_.InitParams(nh_, color_info_);

        if (lidar_buffer_.size() < 10)
        // if (lidar_buffer_.empty())
            return;

        LOG(INFO) << "Localization Initing!";
        PointCloudType::Ptr cloud_xyzi(new PointCloudType());
        
        mtx_buffer_.lock();
        for (int i = 0; i < 10; i++)
        {
            *cloud_xyzi += *(lidar_buffer_.back());
            lidar_buffer_.pop_back();
        }
        // cloud_xyzi = lidar_buffer_.back();
        lidar_buffer_.clear();
        time_buffer_.clear();
        imu_buffer_.clear();
        imu_buf_.clear();
        mtx_buffer_.unlock();

        int size = cloud_xyzi->size();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>(size, 1));
        int i = 0;
        for (const PointType& point_xyzi : *cloud_xyzi)
        {
            common::V3D p_lidar(point_xyzi.x, point_xyzi.y, point_xyzi.z);
            common::V3D p_BOT(R_wrt_ * p_lidar + T_wrt_);

            pcl::PointXYZ point_xyz;
            point_xyz.x = p_BOT(0);
            point_xyz.y = p_BOT(1);
            point_xyz.z = p_BOT(2);
            cloud_xyz->points[i] = point_xyz;
            i++;
        }

        Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
        if (relocalization_.InitExtrinsic(result, cloud_xyz))
        {
            icp_R_wrt_ = result.rotation();
            icp_T_wrt_ = result.translation();
            R_wrt_ = result.rotation() * R_wrt_;
            T_wrt_ = result.rotation() * T_wrt_ + result.translation();
            R_wrt_inv_ = R_wrt_.inverse();
            init_localization_ = false;
            // relocalization_.clear();
            LOG(INFO) << "\033[1;32m----> Init Localization Finished.\033[0m";
            lidar_buffer_.clear();
            time_buffer_.clear();
            imu_buffer_.clear();
            imu_buf_.clear();
        }
        else
        {
            LOG(WARNING) << "----> Init Localization Failed!";
        }

        return;
    }

    if (!SyncPackages()) {
        if (flg_first_odom_ && IMUUpdate())
        {
            SetPosestamp(odom_, state_imu_, last_imu_->angular_velocity);
            odom_.header.stamp = ros::Time().fromSec(last_imu_->header.stamp.toSec());  // ros::Time().fromSec(lidar_end_time_);
            if (pub_odom_aft_mapped_)
                PublishOdometry(pub_odom_aft_mapped_, odom_);
            PublishVelocity(pub_vel_);
        }
        return;
    }

    /// IMU process, kf prediction, undistortion
    p_imu_->Process(measures_, kf_, scan_undistort_);
    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return;
    }

    /// the first scan
    if (flg_first_scan_) {
        ivox_->AddPoints(scan_undistort_->points);
        first_lidar_time_ = measures_.lidar_bag_time_;
        flg_first_scan_ = false;
        return;
    }
    flg_EKF_inited_ = (measures_.lidar_bag_time_ - first_lidar_time_) >= options::INIT_TIME;

    /// downsample
    Timer::Evaluate(
        [&, this]() {
            voxel_scan_.setInputCloud(scan_undistort_);
            voxel_scan_.filter(*scan_down_body_);
        },
        "Downsample PointCloud");

    int cur_pts = scan_down_body_->size();
    if (cur_pts < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        return;
    }
    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, true);
    plane_coef_.resize(cur_pts, common::V4F::Zero());

    // ICP and iterated Kalman filter update
    Timer::Evaluate(
        [&, this]() {
            // iterated state estimation
            double solve_H_time = 0;
            // update the observation model, will call nn and point-to-plane residual computation
            kf_.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
            // save the state
            state_point_ = kf_.get_x();
            euler_cur_ = SO3ToEuler(state_point_.rot);
            pos_lidar_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;
        },
        "IEKF Solve and Update");

    // update local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    state_imu_ = state_point_;
    imu_buf_ = imu_buffer_;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*(measures_.imu_.back())));
    msg->header.stamp = ros::Time().fromSec(measures_.lidar_end_time_);
    last_imu_ = msg;
    flg_first_odom_ = true;

    // static int cnt = 0;
    // if (cnt > 10)
    // {
    //     LOG(INFO) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " downsamp " << cur_pts
    //               << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_;
    //     cnt = 0;
    // }
    // cnt ++;

    if (scan_pub_en_ || pcd_save_en_) {
        PublishFrameWorld();
    }
    if (use_icp_ || (scan_pub_en_ && scan_body_pub_en_)) {
        PublishFrameBody(pub_laser_cloud_body_);
    }
    if (scan_pub_en_ && scan_effect_pub_en_) {
        PublishFrameEffectWorld(pub_laser_cloud_effect_world_);
    }

    IMUUpdate();
    // Debug variables
    frame_num_++;
}

void LaserMapping::StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    mtx_buffer_.lock();
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            PointCloudType::Ptr ptr(new PointCloudType());
            preprocess_->Process(msg, ptr);
            lidar_buffer_.push_back(ptr);
            time_buffer_.push_back(msg->header.stamp.toSec());
            last_timestamp_lidar_ = msg->header.stamp.toSec();
        },
        "Preprocess (Standard)");
    mtx_buffer_.unlock();
}

void LaserMapping::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    mtx_buffer_.lock();
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
                LOG(WARNING) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            last_timestamp_lidar_ = msg->header.stamp.toSec();

            if (!time_sync_en_ && abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0 && !imu_buffer_.empty() &&
                !lidar_buffer_.empty()) {
                LOG(INFO) << "IMU and LiDAR not Synced, IMU time: " << last_timestamp_imu_
                          << ", lidar header time: " << last_timestamp_lidar_;
            }

            if (time_sync_en_ && !timediff_set_flg_ && abs(last_timestamp_lidar_ - last_timestamp_imu_) > 1 &&
                !imu_buffer_.empty()) {
                timediff_set_flg_ = true;
                timediff_lidar_wrt_imu_ = last_timestamp_lidar_ + 0.1 - last_timestamp_imu_;
                LOG(INFO) << "Self sync IMU and LiDAR, time diff is " << timediff_lidar_wrt_imu_;
            }

            PointCloudType::Ptr ptr(new PointCloudType());
            preprocess_->Process(msg, ptr);
            lidar_buffer_.emplace_back(ptr);
            time_buffer_.emplace_back(last_timestamp_lidar_);
        },
        "Preprocess (Livox)");

    mtx_buffer_.unlock();
}

void LaserMapping::IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in) {
    publish_count_++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    if (abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en_) {
        msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu_ + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer_.lock();
    if (timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
        imu_buf_.clear();
    }

    last_timestamp_imu_ = timestamp;
    imu_buffer_.emplace_back(msg);
    imu_buf_.emplace_back(msg);
    mtx_buffer_.unlock();

    static uint8_t cnt = 0;
    cnt++;
    if (cnt % 50 == 0)
    {
        sensor_msgs::Imu record_imu = *msg_in;
        pub_record_imu_.publish(record_imu);
    }
}

void LaserMapping::ColorInfoCallBack(const std_msgs::Bool::ConstPtr &msg_in)
{
    if (color_init_)
        return;

    color_info_ = msg_in->data;
    color_init_ = true;
}

void LaserMapping::PrintState(const state_ikfom &s) {
    LOG(INFO) << "state r: " << s.rot.coeffs().transpose() << ", t: " << s.pos.transpose()
              << ", off r: " << s.offset_R_L_I.coeffs().transpose() << ", t: " << s.offset_T_L_I.transpose();
}

void LaserMapping::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    auto max_size = std::min(points_to_add.max_size(), point_no_need_downsample.max_size());
    int cur_pts = scan_down_body_->size();

    if (cur_pts > max_size)
    {
        cur_pts = max_size;
        scan_down_body_->resize(cur_pts);
    }

    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(std::execution::unseq, index.begin(), index.end(), [&](const size_t &i) {
        /* transform to world frame */
        PointBodyToWorld(&(scan_down_body_->points[i]), &(scan_down_world_->points[i]));

        /* decide if need add to map */
        PointType &point_world = scan_down_world_->points[i];
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector &points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = common::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= options::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < options::NUM_MATCH_POINTS; readd_i++) {
                    if (common::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ivox_->AddPoints(points_to_add);
            ivox_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void LaserMapping::ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
    int cnt_pts = scan_down_body_->size();

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    Timer::Evaluate(
        [&, this]() {
            auto R_wl = (s.rot * s.offset_R_L_I).cast<float>();
            auto t_wl = (s.rot * s.offset_T_L_I + s.pos).cast<float>();

            /** closest surface search and residual computation **/
            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                PointType &point_body = scan_down_body_->points[i];
                PointType &point_world = scan_down_world_->points[i];

                /* transform to world frame */
                common::V3F p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto &points_near = nearest_points_[i];
                if (ekfom_data.converge) {
                    /** Find the closest surfaces in the map **/
                    ivox_->GetClosestPoint(point_world, points_near, options::NUM_MATCH_POINTS);
                    point_selected_surf_[i] = points_near.size() >= options::MIN_NUM_MATCH_POINTS;
                    if (point_selected_surf_[i]) {
                        point_selected_surf_[i] =
                            common::esti_plane(plane_coef_[i], points_near, options::ESTI_PLANE_THRESHOLD);
                    }
                }

                if (point_selected_surf_[i]) {
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);

                    bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                    if (valid_corr) {
                        point_selected_surf_[i] = true;
                        residuals_[i] = pd2;
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    effect_feat_num_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_num_] = plane_coef_[i];
            corr_pts_[effect_feat_num_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_num_][3] = residuals_[i];

            effect_feat_num_++;
        }
    }
    corr_pts_.resize(effect_feat_num_);
    corr_norm_.resize(effect_feat_num_);

    if (effect_feat_num_ < 1) {
        ekfom_data.valid = false;
        LOG(WARNING) << "No Effective Points!";
        return;
    }

    Timer::Evaluate(
        [&, this]() {
            /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
            ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num_, 12);  // 23
            ekfom_data.h.resize(effect_feat_num_);

            index.resize(effect_feat_num_);
            const common::M3F off_R = s.offset_R_L_I.toRotationMatrix().cast<float>();
            const common::V3F off_t = s.offset_T_L_I.cast<float>();
            const common::M3F Rt = s.rot.toRotationMatrix().transpose().cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                common::V3F point_this_be = corr_pts_[i].head<3>();
                common::M3F point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
                common::V3F point_this = off_R * point_this_be + off_t;
                common::M3F point_crossmat = SKEW_SYM_MATRIX(point_this);

                /*** get the normal vector of closest surface/corner ***/
                common::V3F norm_vec = corr_norm_[i].head<3>();

                /*** calculate the Measurement Jacobian matrix H ***/
                common::V3F C(Rt * norm_vec);
                common::V3F A(point_crossmat * C);

                if (extrinsic_est_en_) {
                    common::V3F B(point_be_crossmat * off_R.transpose() * C);
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0],
                        B[1], B[2], C[0], C[1], C[2];
                } else {
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0;
                }

                /*** Measurement: distance to the closest surface/corner ***/
                ekfom_data.h(i) = -corr_pts_[i][3];
            });
        },
        "    ObsModel (IEKF Build Jacobian)");
}

/////////////////////////////////////  debug save / show /////////////////////////////////////////////////////

void LaserMapping::PublishPath(const ros::Publisher pub_path) {
    SetPosestamp(msg_body_pose_, state_imu_);
    msg_body_pose_.header.stamp = ros::Time().fromSec(lidar_end_time_);
    msg_body_pose_.header.frame_id = "odom";

    /*** if path is too large, the rvis will crash ***/
    path_.poses.push_back(msg_body_pose_);
    if (run_in_offline_ == false) {
        pub_path.publish(path_);
    }
}

void LaserMapping::PublishOdometry(const ros::Publisher &pub_odom_aft_mapped, nav_msgs::Odometry &odom_lidar) {
    
    odom_lidar.header.frame_id = "odom";
    odom_lidar.child_frame_id = "lidar_link";

    // TODO:对速度角速度作坐标变换
    nav_msgs::Odometry odom_base;
    odom_base.header.frame_id = "odom";
    odom_base.child_frame_id = "base_link";
    odom_base.header.stamp = odom_lidar.header.stamp;
    odom_base.twist = odom_lidar.twist;

    // R
    Eigen::Quaterniond eigen_quat(odom_lidar.pose.pose.orientation.w, 
                                  odom_lidar.pose.pose.orientation.x, 
                                  odom_lidar.pose.pose.orientation.y, 
                                  odom_lidar.pose.pose.orientation.z);
    common::M3D init_R_lidar = eigen_quat.toRotationMatrix();
    common::M3D MAP_R_BOT = BOT_R_wrt_IMU_ * init_R_lidar * IMU_R_wrt_BOT_;
    if (use_icp_)
        MAP_R_BOT = icp_R_wrt_ * MAP_R_BOT;

    // 转换为tf与odom
    Eigen::Quaterniond q(MAP_R_BOT);
    tf::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
    tf::quaternionTFToMsg(tf_q, odom_base.pose.pose.orientation);

    // T
    common::V3D init_T_lidar(odom_lidar.pose.pose.position.x, 
                             odom_lidar.pose.pose.position.y, 
                             odom_lidar.pose.pose.position.z);
    common::V3D MAP_T_lidar(BOT_R_wrt_IMU_ * init_T_lidar + BOT_T_wrt_IMU_);
    common::V3D MAP_T_BOT(MAP_T_lidar - MAP_R_BOT * BOT_T_wrt_IMU_);
    if (use_icp_)
        MAP_T_BOT = icp_R_wrt_ * MAP_T_BOT + icp_T_wrt_;

    odom_base.pose.pose.position.x = MAP_T_BOT(0);
    odom_base.pose.pose.position.y = MAP_T_BOT(1);
    odom_base.pose.pose.position.z = MAP_T_BOT(2);

    // twist linear // 暂时未用 未消除离心力
    // common::V3D init_V(odom_lidar.twist.twist.linear.x, 
    //                    odom_lidar.twist.twist.linear.y, 
    //                    odom_lidar.twist.twist.linear.z);    
    // common::V3D MAP_V(BOT_R_wrt_IMU_ * init_V);

    // odom_base.twist.twist.linear.x = MAP_V(0);
    // odom_base.twist.twist.linear.y = MAP_V(1);
    // odom_base.twist.twist.linear.z = MAP_V(2);

    // pub_odom_aft_mapped.publish(odom_base);
    auto P = kf_.get_P();
    for (int i = 0; i < 6; i++) {
        int k = i < 3 ? i + 3 : i - 3;
        odom_lidar.pose.covariance[i * 6 + 0] = P(k, 3);
        odom_lidar.pose.covariance[i * 6 + 1] = P(k, 4);
        odom_lidar.pose.covariance[i * 6 + 2] = P(k, 5);
        odom_lidar.pose.covariance[i * 6 + 3] = P(k, 0);
        odom_lidar.pose.covariance[i * 6 + 4] = P(k, 1);
        odom_lidar.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(odom_base.pose.pose.position.x, 
                                    odom_base.pose.pose.position.y,
                                    odom_base.pose.pose.position.z));
    transform.setRotation(tf_q);
    br.sendTransform(tf::StampedTransform(transform, odom_base.header.stamp, "odom", "base_link"));

    static uint8_t cnt = 0;
    cnt++;
    if (cnt % 50 == 0)
    {
        geometry_msgs::PoseStamped odom_pose;
        odom_pose.pose = odom_base.pose.pose;
        odom_pose.header.stamp = ros::Time::now();
        odom_pose.header.frame_id = "map";
        pub_record_odom_.publish(odom_pose);
    }

    tf::Pose tf_odom_pose;
    tf::poseMsgToTF(odom_base.pose.pose, tf_odom_pose);
    tf::StampedTransform odom_to_map_tf_stamp;
    static tf::TransformListener tf_listener;
    try
    {
      tf_listener.lookupTransform("map", "odom", ros::Time(0), odom_to_map_tf_stamp);
      tf_odom_pose = odom_to_map_tf_stamp * tf_odom_pose;
    }
    catch (tf::TransformException ex){
      ROS_WARN("Tracking odom TF lookup: %s",ex.what());
      return;
    }

    tf::poseTFToMsg(tf_odom_pose, odom_base.pose.pose);
    odom_base.header.frame_id = "map";
    pub_odom_aft_mapped.publish(odom_base);

}

void LaserMapping::PublishFrameWorld() {
    if (!(run_in_offline_ == false && scan_pub_en_) && !pcd_save_en_) {
        return;
    }

    PointCloudType::Ptr laserCloudWorld;
    if (dense_pub_en_) {
        PointCloudType::Ptr laserCloudFullRes(scan_undistort_);
        int size = laserCloudFullRes->points.size();
        laserCloudWorld.reset(new PointCloudType(size, 1));
        for (int i = 0; i < size; i++) {
            PointBodyToMap(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }
    } else {
        laserCloudWorld = scan_down_world_;
    }

    if (run_in_offline_ == false && scan_pub_en_) {
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
        laserCloudmsg.header.frame_id = "odom";
        pub_laser_cloud_world_.publish(laserCloudmsg);
        publish_count_ -= options::PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en_) {
        *pcl_wait_save_ += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save_->size() > 0 && pcd_save_interval_ > 0 && scan_wait_num >= pcd_save_interval_) {
            pcd_index_++;
            std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/scans_") + std::to_string(pcd_index_) +
                                       std::string(".pcd"));
            pcl::PCDWriter pcd_writer;
            LOG(INFO) << "current scan saved to /PCD/" << all_points_dir;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);
            pcl_wait_save_->clear();
            scan_wait_num = 0;
        }
    }
}

void LaserMapping::PublishFrameBody(const ros::Publisher &pub_laser_cloud_body) {
    int size = scan_undistort_->points.size();
    // PointCloudType::Ptr laser_cloud_imu_body(new PointCloudType(size, 1));
    laser_cloud_imu_body_->resize(size);

    for (int i = 0; i < size; i++)
    {
        PointBodyLidarToIMU(&scan_undistort_->points[i], &laser_cloud_imu_body_->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laser_cloud_imu_body_, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
    laserCloudmsg.header.frame_id = "body";
    pub_laser_cloud_body.publish(laserCloudmsg);
    publish_count_ -= options::PUBFRAME_PERIOD;
}

void LaserMapping::PublishFrameEffectWorld(const ros::Publisher &pub_laser_cloud_effect_world) {
    int size = corr_pts_.size();
    PointCloudType::Ptr laser_cloud(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++) {
        PointBodyToMap(corr_pts_[i].head<3>(), &laser_cloud->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laser_cloud, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
    laserCloudmsg.header.frame_id = "odom";
    pub_laser_cloud_effect_world.publish(laserCloudmsg);
    publish_count_ -= options::PUBFRAME_PERIOD;
}

void LaserMapping::PublishVelocity(const ros::Publisher &pub_vel)
{
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = odom_.header.stamp;
    msg.header.frame_id = "odom";
    msg.twist = odom_.twist.twist;
    pub_vel.publish(msg);
}

void LaserMapping::Savetrajectory(const std::string &traj_file) {
    std::ofstream ofs;
    ofs.open(traj_file, std::ios::out);
    if (!ofs.is_open()) {
        LOG(ERROR) << "Failed to open traj_file: " << traj_file;
        return;
    }

    ofs << "#timestamp x y z q_x q_y q_z q_w" << std::endl;
    for (const auto &p : path_.poses) {
        ofs << std::fixed << std::setprecision(6) << p.header.stamp.toSec() << " " << std::setprecision(15)
            << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << " " << p.pose.orientation.x
            << " " << p.pose.orientation.y << " " << p.pose.orientation.z << " " << p.pose.orientation.w << std::endl;
    }

    ofs.close();
}

///////////////////////////  private method /////////////////////////////////////////////////////////////////////
void LaserMapping::SetPosestamp(nav_msgs::Odometry &out, state_ikfom state, geometry_msgs::Vector3 angvel) {
    out.pose.pose.position.x = state.pos(0);
    out.pose.pose.position.y = state.pos(1);
    out.pose.pose.position.z = state.pos(2);
    out.pose.pose.orientation.x = state.rot.coeffs()[0];
    out.pose.pose.orientation.y = state.rot.coeffs()[1];
    out.pose.pose.orientation.z = state.rot.coeffs()[2];
    out.pose.pose.orientation.w = state.rot.coeffs()[3];
    out.twist.twist.linear.x = state.vel(0);
    out.twist.twist.linear.y = state.vel(1);
    out.twist.twist.linear.z = state.vel(2);
    out.twist.twist.angular.x = angvel.x - state.bg(0);
    out.twist.twist.angular.x = angvel.y - state.bg(1);
    out.twist.twist.angular.x = angvel.z - state.bg(2);
}

void LaserMapping::SetPosestamp(geometry_msgs::PoseStamped &out ,state_ikfom state) {
    out.pose.position.x = state.pos(0);
    out.pose.position.y = state.pos(1);
    out.pose.position.z = state.pos(2);
    out.pose.orientation.x = state.rot.coeffs()[0];
    out.pose.orientation.y = state.rot.coeffs()[1];
    out.pose.orientation.z = state.rot.coeffs()[2];
    out.pose.orientation.w = state.rot.coeffs()[3];
}

void LaserMapping::PointBodyToMap(const PointType *pi, PointType *const po) {
    common::V3D p_body(pi->x, pi->y, pi->z);
    // common::V3D p_global(R_wrt_ * (state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
    //                      state_point_.pos) + T_wrt_);
    common::V3D p_global(R_wrt_ * (state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos) + T_wrt_);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LaserMapping::PointBodyToMap(const common::V3F &pi, PointType *const po) {
    common::V3D p_body(pi.x(), pi.y(), pi.z());
    // common::V3D p_global(R_wrt_ * (state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
    //                      state_point_.pos) + T_wrt_);
    common::V3D p_global(R_wrt_ * (state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos) + T_wrt_);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = std::abs(po->z);
}

void LaserMapping::PointBodyToWorld(const PointType *pi, PointType *const po) {
    common::V3D p_body(pi->x, pi->y, pi->z);
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LaserMapping::PointBodyToWorld(const common::V3F &pi, PointType *const po) {
    common::V3D p_body(pi.x(), pi.y(), pi.z());
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos);


    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = std::abs(po->z);
}

void LaserMapping::PointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
    common::V3D p_body_lidar(pi->x, pi->y, pi->z);
    common::V3D p_body_imu(state_point_.offset_R_L_I * p_body_lidar + state_point_.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void LaserMapping::Finish() {
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save_->size() > 0 && pcd_save_en_) {
        std::string file_name = std::string("scans.pcd");
        std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        LOG(INFO) << "current scan saved to /PCD/" << file_name;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);
    }

    LOG(INFO) << "finish done";
}
}  // namespace faster_lio