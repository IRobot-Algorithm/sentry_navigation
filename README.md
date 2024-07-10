# Sentry Navigation
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
  for file in /home/nav_ws/thirdparty/*; do \
      if [ -f "$file" ]; then \
          case "$file" in \
              *.tar.gz) tar -xzf "$file" -C /home/nav_ws/thirdparty ;; \
              *.zip) unzip "$file" -d /home/nav_ws/thirdparty ;; \
              *.tar.bz2) tar -xjf "$file" -C /home/nav_ws/thirdparty ;; \
              *.tar.xz) tar -xJf "$file" -C /home/nav_ws/thirdparty ;; \
          esac; \
          rm "$file"; \
      fi; \
  done
  find /home/nav_ws/thirdparty -name "install.sh" -type f -exec bash -c 'cd "$(dirname "{}")" && chmod 777 install.sh && ./install.sh' \;
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

  
