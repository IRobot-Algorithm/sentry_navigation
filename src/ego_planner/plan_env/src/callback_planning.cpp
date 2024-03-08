void GridMap::cloudSegmentationCallback_PLANNING(const sensor_msgs::PointCloud2ConstPtr &cloud_segmentation) {
    my_count+=1;
    ros::Time time_begin = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZ> latest_cloud;
    pcl::fromROSMsg(*cloud_segmentation, latest_cloud);

    if (!md_.has_odom_)
    {
        std::cout << "no odom!" << std::endl;
        return;
    }

    if (latest_cloud.points.size() == 0)
        return;

    if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2)))
        return;

    md_.lidar_proj_points_ = latest_cloud;

//  this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
//                    md_.camera_pos_ + mp_.local_update_range_);

    pcl::PointXYZ pt;
    Eigen::Vector3d p3d, p3d_inf;

    int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
    int inf_step_z = 1;
    //cout<<"Inflate size : "<<inf_step<<endl;
    double max_x, max_y, max_z, min_x, min_y, min_z;

    min_x = mp_.map_max_boundary_(0);
    min_y = mp_.map_max_boundary_(1);
    min_z = mp_.map_max_boundary_(2);

    max_x = mp_.map_min_boundary_(0);
    max_y = mp_.map_min_boundary_(1);
    max_z = mp_.map_min_boundary_(2);

    //if(mp_.map_type_==PLANNING)
    //{
    //ROS_INFO("Point cloud num : %d",latest_cloud.points.size());
    for (size_t i = 0; i < latest_cloud.points.size(); ++i)
    {
        pt = latest_cloud.points[i];
        p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

        /* point inside update range */
        Eigen::Vector3d devi = p3d - md_.camera_pos_;
        Eigen::Vector3i inf_pt;

        if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
            fabs(devi(2)) < mp_.local_update_range_(2))
        {

            /* inflate the point */
            for (int x = -inf_step; x <= inf_step; ++x)
                for (int y = -inf_step; y <= inf_step; ++y)
                    for (int z = -inf_step_z; z <= inf_step_z; ++z)
                    {
                        p3d_inf(0) = pt.x + x * mp_.resolution_;
                        p3d_inf(1) = pt.y + y * mp_.resolution_;
                        p3d_inf(2) = pt.z + z * mp_.resolution_;

                        max_x = max(max_x, p3d_inf(0));
                        max_y = max(max_y, p3d_inf(1));
                        max_z = max(max_z, p3d_inf(2));

                        min_x = min(min_x, p3d_inf(0));
                        min_y = min(min_y, p3d_inf(1));
                        min_z = min(min_z, p3d_inf(2));

                        posToIndex(p3d_inf, inf_pt);
                        // ROS_INFO("Run here! 2");
                        if (!isInMap(inf_pt))
                            continue;

                        int idx_inf = toAddress(inf_pt);

                        md_.occupancy_buffer_inflate_segmentation_[idx_inf] = 1;
                    }
        }
    }

    boundIndex(md_.local_bound_min_);
    boundIndex(md_.local_bound_max_);

    Eigen::Vector3i temp_idx;
    Eigen::Vector3i temp_pos;
    int count=0;
    for(int x=md_.local_bound_min_(0);x<md_.local_bound_max_(0);x++)
        for(int y=md_.local_bound_min_(1);y<md_.local_bound_max_(1);y++)
            for(int z=md_.local_bound_min_(2);z<md_.local_bound_max_(2);z++)
            {
                temp_idx<<x,y,z;
                if(md_.occupancy_buffer_inflate_segmentation_[toAddress(temp_idx)]==1&&z<=(md_.camera_pos_(2)+3))
                {
                    count+=1;
                }

                if(z==(md_.local_bound_max_(2)-1))
                {
                    if(count>=1)
                    {
                        md_.occupancy_buffer_inflate_2d_[toAddress2d(x,y)]=1;
                    }
                    count =0;
                }
            }

    publishMap2D();

}
