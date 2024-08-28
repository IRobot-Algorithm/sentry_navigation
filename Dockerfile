FROM ghcr.io/irobot-algorithm/sentry_navigation/environment:latest

# Create workspace
RUN mkdir -p /home/nav_ws
WORKDIR /home/nav_ws

# Copy files
ADD src /home/nav_ws/src
ADD build.sh /home/nav_ws

# Build the workspace
RUN chmod 777 build.sh && \
    ./build.sh

# Run test
RUN exec bash && \
    source /opt/ros/noetic/setup.sh && \
    catkin_test_results build