FROM osrf/ros:humble-desktop

# Install Gazebo
RUN sudo apt-get update && \
    sudo apt-get install -y gazebo11 libgazebo11-dev

# Install RViz
RUN sudo apt-get update && \
    sudo apt-get install -y ros-humble-rviz2

# Set environment variables for GUI
ENV DISPLAY=host.docker.internal VOLUME=/tmp/.X11-unix:/tmp/.X11-unix
