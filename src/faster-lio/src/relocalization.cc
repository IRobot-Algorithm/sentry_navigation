#include "relocalization.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

Relocalization::Relocalization()
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> ICP location started.\033[0m");

    // 指针的初始化
    cloud_map_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    cloud_scan_ = boost::shared_ptr<PointCloudT>(new PointCloudT());

}

Relocalization::~Relocalization()
{
}

/*
 * 的参数初始化
 */
void Relocalization::InitParams(ros::NodeHandle &nh)
{
    nh.param<std::string>("icp/pcd_path", pcd_path_);   //scan与匹配的最大时间间隔

    //ICP匹配相关参数
    nh.param<double>("icp/AGE_THRESHOLD", AGE_THRESHOLD_, 1);   //scan与匹配的最大时间间隔
    nh.param<double>("icp/ANGLE_UPPER_THRESHOLD", ANGLE_UPPER_THRESHOLD_, 10);    //最大变换角度
    nh.param<double>("icp/ANGLE_THRESHOLD", ANGLE_THRESHOLD_, 0.01);    //最小变换角度
    nh.param<double>("icp/DIST_THRESHOLD", DIST_THRESHOLD_, 0.01);    //最小变换距离
    nh.param<double>("icp/SCORE_THRESHOLD_MAX", SCORE_THRESHOLD_MAX_, 0.1);    //达到最大迭代次数或者到达差分阈值后后，代价仍高于此值，认为无法收敛,自适应使用
    nh.param<double>("icp/Point_Quantity_THRESHOLD", Point_Quantity_THRESHOLD_, 200);   //点云数阈值,低于此值不匹配
    nh.param<double>("icp/Maximum_Iterations", Maximum_Iterations_, 100);   //ICP中的最大迭代次数

    //体素滤波的边长
    nh.param<double>("icp/VoxelGridRemoval_LeafSize", VoxelGridRemoval_LeafSize_, 0.05); 

    //迭代障碍物去除
    //如果雷达点云中点在地图点云最近点大于此值，就认为该点为障碍点，有最大和最小值，会随着icp迭代的SCORE值按比例进行更新
    nh.param<double>("icp/ObstacleRemoval_Distance_Max", ObstacleRemoval_Distance_Max_, 2);     //最大距离

    if (pcl::io::loadPCDFile<PointT>(pcd_path_, *cloud_scan_) == -1)
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
    PointCloudVoxelGridRemoval(cloud_scan_, VoxelGridRemoval_LeafSize_);
    PointCloudObstacleRemoval(cloud_map_, cloud_scan_, ObstacleRemoval_Distance_Max_);
    // scan_pointcloud_publisher_.publish(cloud_scan_);
    // map_pointcloud_publisher_.publish(cloud_map_);   //发布pointcloud地图

    //使用ICP进行点云匹配
    if (!ScanMatchWithICP(match_result, cloud_scan_, cloud_map_))
    {
        // scan_initialized_ = false;  //数据错误，需要重新初始化
        return false; 
    }

    return true;
}

/*
 * 使用ICP进行帧间位姿的计算
 */
bool Relocalization::ScanMatchWithICP(Eigen::Isometry3d &trans , PointCloudT::Ptr &cloud_scan, PointCloudT::Ptr &cloud_map)
{

    icp_.setTransformationEpsilon (1e-6);    //为中止条件设置最小转换差异
    icp_.setEuclideanFitnessEpsilon(1e-6);
    icp_.setMaxCorrespondenceDistance(5);
    icp_.setMaximumIterations (Maximum_Iterations_);    //设置匹配迭代最大次数

    // ICP 输入数据,输出数据的设置,还可以进行参数配置,这里使用默认参宿
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
    Eigen::Affine3f transfrom;
    transfrom = icp_.getFinalTransformation();

    // 将Eigen::Affine3f转换成x, y, theta, 并打印出来
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transfrom, x, y, z, roll, pitch, yaw);
    // std::cout << "ICP transfrom: (" << x << ", " << y << ", " << yaw << ")" << std::endl;

    double tranDist = sqrt(x*x + y*y);
    double angleDist = abs(yaw);

    if (if_debug_)
    {
        std::cout<< "tranDist:" << tranDist << " angleDist: " << angleDist 
        << " score: " << icp_.getFitnessScore() << std::endl;
    }

    if (if_debug_)
    {
        std::cout << "cloud_scan->points.size() : " << cloud_scan->points.size() << std::endl;
    }
    
    std::pair<double,double> coord = {x,y};
    // 如果变换小于一定值，不发布结果，退出
    if(tranDist < DIST_THRESHOLD_ && angleDist < ANGLE_THRESHOLD_ ||
         cloud_scan->points.size() < Point_Quantity_THRESHOLD_ )
    {
        if(if_debug_)
        {
            // \033[1;33m，\033[0m 终端显示成黄色
            std::cout << "\033[1;33m" << "Distance or point_Quantity out of threshold" << "\033[0m" << std::endl;
            std::cout << "\033[1;33m"  << "tranDist:" << tranDist << " angleDist: " << angleDist 
            << " score: " << icp_.getFitnessScore() << "\033[0m" << std::endl;
        }
        return false;
    }

    //如果匹配结果不满足条件，退出
    if(angleDist > ANGLE_UPPER_THRESHOLD_ || icp_.getFitnessScore() > SCORE_THRESHOLD_MAX_)
    {
        if(if_debug_)
        {
            // \033[1;33m，\033[0m 终端显示成黄色
            std::cout << "\033[1;33m" << "result out of threshold" << "\033[0m" << std::endl;
            std::cout << "\033[1;33m"  << "tranDist:" << tranDist << " angleDist: " << angleDist 
            << " score: " << icp_.getFitnessScore() << "\033[0m" << std::endl;
            std::cout << "location_loss_num_: " << location_loss_num_ << std::endl;
        }
        return false;
    }
    location_loss_num_ = 0;

    trans.matrix() = transfrom.matrix().cast<double>();      //Matrix4f类型转换为Isometry3d类型

    // icp_pointcloud_publisher_.publish(pointcloud_result);   //发布匹配结果
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
    int K =1;   // k近邻收索

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

    // cloud_removaled->width = cloud_removaled->points.size() ;
    // cloud_removaled->height = 1;
    // cloud_removaled->is_dense = false; // contains nans

    // std_msgs::Header header;
    // header.stamp = ros::Time::now();
    // // header.frame_id = lidar_frame_;
    // header.frame_id = "map";
    // cloud_removaled->header = pcl_conversions::toPCL(header);
    // removal_pointcloud_publisher_.publish(cloud_removaled);
    // std::cout<<"size of clound ObstacleRemovaled : " << cloud_scan->points.size() << std::endl;

}

/**
 * 对点云进行离群点滤波剔除离群点
 */
void Relocalization::PointCloudOutlierRemoval(PointCloudT::Ptr &cloud_scan)
{
    // std::cout<<"size of clound UnOutlierRemoval : " << cloud_scan->points.size() << std::endl;

    /* 声明 离群点滤波 后 的点云 */
    // pcl::PointCloud<PointT>::Ptr cloud_OutRemove_filtered (new pcl::PointCloud<PointT>);
 
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
 
 
    /* 打印滤波前后的点数 */
    // std::cout << "size of clound OutlierRemovaled : " << cloud_scan->points.size() << std::endl;
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
