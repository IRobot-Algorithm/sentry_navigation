# Sentry Navigation

[![Build Test](https://github.com/IRobot-Algorithm/sentry_navigation/actions/workflows/build-CI.yaml/badge.svg)](https://github.com/IRobot-Algorithm/sentry_navigation/actions/workflows/build-CI.yaml)

## introduction

本仓库为2024哨兵导航仓库

main为双头构型，spare-sentry为单头构型

- 决策

  - **state_processing**

    维护一个状态机执行更上层的决策指令

- 定位

  - **faster_lio**

    在原有代码基础上利用imu数据结合内部的esekf前向预测，获得了大约200hz的定位频率，增加转换矩阵适应雷达的不同安装

  - **fast_gicp**

    将实时采集的pcd与建图获得的pcd进行icp，维护map->odom的回环检测；经赛场测试，官方uwb定位精度极高，因此选择在uwb和定位高度不匹配时在此线程重启lio进行重定位

- 感知

  - **pointcloud_repub**

    裁切进入lio前的点云，变换lio去畸变后的点云到世界系，融合其他传感器（如d435)点云

  - **terrain_anlysis**

    可根据雷达fov选择每次分析时更新地图栅格的区域，适应各种雷达安装方式

  - **obstacle_generator**

    根据赛场已知地图，将静态障碍物发布为障碍物点云供规划使用

- 规划

  - **far_planner**

    以拓扑图为基础的全局规划，在动态场景下效果较好，规划的路径相较A*更加适配下层离线轨迹的local planner

  - **local_planner**

    cmu_exploration的local_planner，以离线轨迹为基础，为适应全向移动在轨迹评分上有部分改动

- 执行

  - **path_follow**

    简单的比例跟随轨迹，为适应赛场决策，目标跟随，云台控制，小陀螺等决策指令也在这里一并处理发出

- 通信

  - **nav_transporter**

    使用usb通信，can通信长时间未维护


## environment

- 下载所需依赖：

  ```shell
  apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
  echo "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" > /etc/apt/sources.list.d/realsense.list
  apt-get update
  DEBIAN_FRONTEND=noninteractive apt-get install -y build-essential cmake libeigen3-dev libpcl-dev libgoogle-glog-dev libudev-dev git libxrandr-dev libxi-dev libxinerama-dev libsdl2-dev libusb-1.0.0-dev
  cp -rf /usr/include/eigen3/Eigen /usr/include/Eigen -R
  ```

- clone该仓库：

  ```shell
  git clone git@github.com:IRobot-Algorithm/sentry_navigation.git
  ```

- 使用`lfs`下载所需的第三方库：

  ```shell
  git lfs install
  git lfs pull
  ```

- 解压并编译安装第三方库：

  ```shell
  for file in ./thirdparty/*; do \
      if [ -f "$file" ]; then \
          case "$file" in \
              *.tar.gz) tar -xzf "$file" -C ./thirdparty ;; \
              *.zip) unzip "$file" -d ./thirdparty ;; \
              *.tar.bz2) tar -xjf "$file" -C ./thirdparty ;; \
              *.tar.xz) tar -xJf "$file" -C ./thirdparty ;; \
          esac; \
          rm "$file"; \
      fi; \
  done
  find ./thirdparty -name "install.sh" -type f -exec bash -c 'cd "$(dirname "{}")" && chmod 777 install.sh && ./install.sh' \;
  ```

- 下载所需ROS第三方库：

  ```shell
  apt-get install -y ros-noetic-rviz ros-noetic-cv-bridge ros-noetic-pcl-ros \
                     ros-noetic-tf-conversions ros-noetic-eigen-conversions \
                     ros-noetic-ddynamic-reconfigure ros-noetic-diagnostic-updater \
                     ros-noetic-image-geometry ros-noetic-costmap-2d \
                     ros-noetic-nav-core ros-noetic-navfn
  ```

## build

- 运行`build.sh`

  ```shell
  ./build.sh
  ```

  

## run

``` shell
source /opt/ros/noetic/setup.bash
source /devel/setup.bash
roslaunch nav_bringup sentry_nav.launch
```

## docker

- 生成image

  ```shell
  docker build . -t sentry_navigation:latest
  ```

- 生成container

  ```shell
  docker run -it sentry_navigation:latest
  ```

- 运行

  ```shell
  source /opt/ros/noetic/setup.bash
  source /devel/setup.bash
  roslaunch nav_bringup sentry_nav.launch
  ```

  
