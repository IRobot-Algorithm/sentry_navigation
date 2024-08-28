FROM ghcr.io/irobot-algorithm/sentry_navigation/environment:latest

# Create workspace
RUN mkdir -p /home/nav_ws
WORKDIR /home/nav_ws

# Copy files
ADD src /home/nav_ws/src
ADD thirdparty /home/nav_ws/thirdparty
ADD build.sh /home/nav_ws

# Install dependencies
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN echo "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" > /etc/apt/sources.list.d/realsense.list
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y build-essential cmake libeigen3-dev libpcl-dev libgoogle-glog-dev libudev-dev git libxrandr-dev libxi-dev libxinerama-dev libsdl2-dev libusb-1.0.0-dev && \
    cp -rf /usr/include/eigen3/Eigen /usr/include/Eigen -R

# Install thirdparty dependencies
RUN for file in /home/nav_ws/thirdparty/*; do \
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
RUN find /home/nav_ws/thirdparty -name "install.sh" -type f -exec bash -c 'cd "$(dirname "{}")" && chmod 777 install.sh && ./install.sh' \;

# # Install ROS dependencies
RUN apt-get install -y ros-noetic-rviz ros-noetic-cv-bridge ros-noetic-pcl-ros \
                       ros-noetic-tf-conversions ros-noetic-eigen-conversions \
                       ros-noetic-ddynamic-reconfigure ros-noetic-diagnostic-updater \
                       ros-noetic-image-geometry ros-noetic-costmap-2d \
                       ros-noetic-nav-core ros-noetic-navfn


# Build the workspace
RUN chmod 777 build.sh && \
    ./build.sh

# Run test
RUN exec bash && \
    source /opt/ros/noetic/setup.sh && \
    catkin_test_results build