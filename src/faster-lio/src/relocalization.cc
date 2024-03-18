#include "relocalization.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#define PI 3.1415927

Relocalization::Relocalization()
{

    // 指针的初始化
    cloud_map_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    cloud_scan_ = boost::shared_ptr<PointCloudT>(new PointCloudT());

}

Relocalization::~Relocalization()
{
}

void Relocalization::clear()
{
    cloud_map_->clear();
    cloud_scan_->clear();
    RGBcloud_map_->clear();
    RGBcloud_scan_->clear();
    RGBcloud_result_->clear();
}

/*
 * 的参数初始化
 */
void Relocalization::InitParams(ros::NodeHandle &nh)
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> ICP location started.\033[0m");

    nh.param<bool>("icp/save_result", save_result_, false);
    nh.param<bool>("icp/pub_result", pub_result_, false);

    nh.param<std::string>("icp/map_pcd_path", map_pcd_path_, "");
    nh.param<std::string>("icp/ori_pcd_path", ori_pcd_path_, "");
    nh.param<std::string>("icp/res_pcd_path", res_pcd_path_, "");

    //ICP匹配相关参数
    nh.param<double>("icp/AGE_THRESHOLD", AGE_THRESHOLD_, 1);   //scan与匹配的最大时间间隔
    nh.param<double>("icp/ANGLE_UPPER_THRESHOLD", ANGLE_UPPER_THRESHOLD_, 10);    //最大变换角度
    nh.param<double>("icp/SCORE_THRESHOLD_MAX", SCORE_THRESHOLD_MAX_, 0.1);    //达到最大迭代次数或者到达差分阈值后后，代价仍高于此值，认为无法收敛,自适应使用
    nh.param<double>("icp/Point_Quantity_THRESHOLD", Point_Quantity_THRESHOLD_, 200);   //点云数阈值,低于此值不匹配
    nh.param<double>("icp/Maximum_Iterations", Maximum_Iterations_, 100);   //ICP中的最大迭代次数

    //体素滤波的边长
    nh.param<double>("icp/VoxelGridRemoval_LeafSize", VoxelGridRemoval_LeafSize_, 0.05); 

    //迭代障碍物去除
    //如果雷达点云中点在地图点云最近点大于此值，就认为该点为障碍点，有最大和最小值，会随着icp迭代的SCORE值按比例进行更新
    nh.param<double>("icp/ObstacleRemoval_Distance_Max", ObstacleRemoval_Distance_Max_, 2);     //最大距离

    if (save_result_ || pub_result_)
    {
        RGBcloud_map_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(new pcl::PointCloud<pcl::PointXYZRGB>);
        RGBcloud_scan_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(new pcl::PointCloud<pcl::PointXYZRGB>);
        RGBcloud_result_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(new pcl::PointCloud<pcl::PointXYZRGB>);

        if (pub_result_)
        {
            pub_map_ = nh.advertise<sensor_msgs::PointCloud2>("/icp_cloud_map", 1);
            pub_scan_ = nh.advertise<sensor_msgs::PointCloud2>("/icp_cloud_scan", 1);
        }
    }
    

    if (pcl::io::loadPCDFile<PointT>(map_pcd_path_, *cloud_map_) == -1)
    {
        PCL_ERROR("Couldn't read PCD file\n");
        return;
    }

}

bool Relocalization::InitExtrinsic(Eigen::Isometry3d &match_result , PointCloudT::Ptr &scan)
{
    cloud_scan_ = scan;
    // 进行离群点滤波，剔除离群点
    // PointCloudOutlierRemoval(cloud_scan_);
    if (cloud_scan_->size() < 100)
    {
        std::cout << "\033[1;33m" << "Too less cloud to relocalization" << "\033[0m" << std::endl;
        return false;
    }
    PointCloudVoxelGridRemoval(cloud_scan_, VoxelGridRemoval_LeafSize_);
    if (cloud_scan_->size() < 100)
    {
        std::cout << "\033[1;33m" << "Too less cloud to relocalization" << "\033[0m" << std::endl;
        return false;
    }
    PointCloudObstacleRemoval(cloud_map_, cloud_scan_, ObstacleRemoval_Distance_Max_);

    //使用ICP进行点云匹配
    if (!ScanMatchWithICP(match_result, cloud_scan_, cloud_map_))
    {
        location_loss_num_++;
        std::cout << "location_loss_num_: " << location_loss_num_ << std::endl;
        return false; 
    }

    location_loss_num_ = 0;
    first_icp_ = true;
    return true;
}

/*
 * 使用ICP进行帧间位姿的计算
 */
bool Relocalization::ScanMatchWithICP(Eigen::Isometry3d &trans , PointCloudT::Ptr &cloud_scan, PointCloudT::Ptr &cloud_map)
{
    if (pub_result_)
    {
        RGBcloud_map_->clear();
        for (const pcl::PointXYZ& point : *cloud_map)
        {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = 128;
            colored_point.g = 128;
            colored_point.b = 128;
            RGBcloud_map_->push_back(colored_point);
        }

        RGBcloud_scan_->clear();
        for (const pcl::PointXYZ& point : *cloud_scan)
        {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = 255;
            colored_point.g = 255;
            colored_point.b = 0;
            RGBcloud_scan_->push_back(colored_point);
        }

        sensor_msgs::PointCloud2 map_msg;
        sensor_msgs::PointCloud2 scan_msg;
        pcl::toROSMsg(*RGBcloud_map_, map_msg);
        pcl::toROSMsg(*RGBcloud_scan_, scan_msg);
        map_msg.header.frame_id = "map";
        scan_msg.header.frame_id = "map";
        pub_map_.publish(map_msg);
        pub_scan_.publish(scan_msg);
    }

    icp_.setTransformationEpsilon (1e-6);    //为中止条件设置最小转换差异
    icp_.setEuclideanFitnessEpsilon(1e-6);
    icp_.setMaxCorrespondenceDistance(5);
    icp_.setMaximumIterations (Maximum_Iterations_);    //设置匹配迭代最大次数

    icp_.setInputSource(cloud_scan);
    icp_.setInputTarget(cloud_map);

    // 开始迭代计算
    pcl::PointCloud<PointT> pointcloud_result;
    icp_.align(pointcloud_result);

    // 如果迭代没有收敛,不进行输出
    if (icp_.hasConverged() == false)
    {
        std::cout << "not Converged" << std::endl;
        return false;
    }

    // 收敛了之后, 获取坐标变换
    Eigen::Affine3f transform;
    transform = icp_.getFinalTransformation();

    // 将Eigen::Affine3f转换成x, y, theta, 并打印出来
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transform, x, y, z, roll, pitch, yaw);

    double tranDist = sqrt(x*x + y*y);

    if (if_debug_)
    {
        std::cout << "ICP transform: (" << x << ", " << y << ", " << z << ")" << std::endl;
        std::cout << "ICP rotation: (" << roll * 180.0 / PI << ", " << pitch * 180.0 / PI << ", " << yaw * 180.0 / PI << ")" << std::endl; 
        std::cout << " score: " << icp_.getFitnessScore() << std::endl;
        std::cout << "cloud_scan->points.size() : " << cloud_scan->points.size() << std::endl;
    }

    if(cloud_scan->points.size() < Point_Quantity_THRESHOLD_ )
    {
        if(if_debug_)
        {
            // \033[1;33m，\033[0m 终端显示成黄色
            std::cout << "\033[1;33m" << "Point_Quantity out of threshold" << "\033[0m" << std::endl;
            std::cout << " score: " << icp_.getFitnessScore() << "\033[0m" << std::endl;
        }
        return false;
    }

    //如果匹配结果不满足条件，退出
    if(icp_.getFitnessScore() > SCORE_THRESHOLD_MAX_)
    {
        if(if_debug_)
        {
            // \033[1;33m，\033[0m 终端显示成黄色
            std::cout << "\033[1;33m" << "result out of threshold" << "\033[0m" << std::endl;
            std::cout << " score: " << icp_.getFitnessScore() << "\033[0m" << std::endl;
        }
        return false;
    }

    trans.matrix() = transform.matrix().cast<double>();      //Matrix4f类型转换为Isometry3d类型

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZ>);
    if ((first_icp_ && save_result_) || pub_result_)
    {
        // 对点云进行变换
        pcl::transformPointCloud(*cloud_scan, *cloud_result, transform);

        RGBcloud_result_->clear();
        for (const pcl::PointXYZ& point : *cloud_result)
        {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = 255;
            colored_point.g = 255;
            colored_point.b = 0;
            RGBcloud_result_->push_back(colored_point);
        }
        
        if (first_icp_ && save_result_)
        {
            pcl::io::savePCDFileBinary(ori_pcd_path_, *RGBcloud_map_ + *RGBcloud_scan_);
            pcl::io::savePCDFileBinary(res_pcd_path_, *RGBcloud_map_ + *RGBcloud_result_);
        }
        if (pub_result_)
        {
            sensor_msgs::PointCloud2 map_msg;
            sensor_msgs::PointCloud2 scan_msg;
            pcl::toROSMsg(*RGBcloud_map_, map_msg);
            pcl::toROSMsg(*RGBcloud_result_, scan_msg);
            map_msg.header.frame_id = "map";
            scan_msg.header.frame_id = "map";
            pub_map_.publish(map_msg);
            pub_scan_.publish(scan_msg);
        }
        RGBcloud_map_->clear();
        RGBcloud_scan_->clear();
        RGBcloud_result_->clear();

    }

    return true;
}


/**
 * 对点云中障碍点进行剔除
 * cloud_map为参考点云，cloud_scan为需要剔除障碍点的点云
 */
void Relocalization::PointCloudObstacleRemoval(PointCloudT::Ptr &cloud_map, PointCloudT::Ptr &cloud_scan, double Distance_Threshold)
{
    PointCloudT::Ptr cloud_removaled(new PointCloudT);;
    // std::cout<<"size of clound UnObstacleRemoval : " << cloud_scan->points.size() << std::endl;

    pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;  //创建kd_tree对象
    kdtree.setInputCloud (cloud_map); //设置搜索空间
    int K = 1;   // k近邻收索

    // for (int i = 0; i < cloud_scan->points.size(); i++)
    int i = 0;
    while(i < cloud_scan->points.size())
    {
        PointT searchPoint = cloud_scan->points[i];

        // k近邻收索
        std::vector<int>pointIdxNKNSearch(K); //存储查询点近邻索引
        std::vector<float>pointNKNSquaredDistance(K); //存储近邻点对应平方距离

        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            if (pointNKNSquaredDistance[0] > Distance_Threshold) //大于阈值认为是障碍点，剔除
            {
                cloud_scan->erase(cloud_scan->begin() + i);

                cloud_removaled->push_back(searchPoint);//从点云最后面插入一点

            }
            else
            {
                i++;
            }
        }
    }

}

/**
 * 对点云进行离群点滤波剔除离群点
 */
void Relocalization::PointCloudOutlierRemoval(PointCloudT::Ptr &cloud_scan)
{

    /* 声明 离群点滤波 的 类实例 */
    pcl::StatisticalOutlierRemoval<PointT> sor_OutRemove;
    /* 设置输入点云 */
    sor_OutRemove.setInputCloud (cloud_scan);
    /* 设置在进行统计时考虑查询点邻近点数 */
    sor_OutRemove.setMeanK (30);
    /* 设置判断是否为离群点 的 阈值  设置为1的 话 表示为：如果一个点的距离超过平均距离一个标准差以上则为离群点 */
    sor_OutRemove.setStddevMulThresh (1.0);
     /* 执行滤波 返回 滤波后 的 点云 */
    sor_OutRemove.filter (*cloud_scan);

}

/**
 * 对点云进行体素滤波剔除离群点
 */
void Relocalization::PointCloudVoxelGridRemoval(PointCloudT::Ptr &cloud_scan, double leafSize)
{
    // std::cout<<"size of clound UnVoxelGridRemoval : " << cloud_scan->points.size() << std::endl;

    /* 声明体素滤波的类实例 */
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud_scan);
    voxel_grid.setLeafSize(leafSize, leafSize, leafSize);//注：体素不一定是正方体，体素可以是长宽高不同的长方体
    voxel_grid.filter(*cloud_scan);
    /* 打印滤波前后的点数 */
    if (if_debug_)
    {
        std::cout << "size of clound VoxelGridRemovaled : " << cloud_scan->points.size() << std::endl;
    }
}
