FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    PIP_NO_CACHE_DIR=1

# ROS 2 / Gazebo 関連
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-xacro \
      ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-vision-msgs \
      ros-jazzy-cv-bridge \
      python3-pip \
      python3-vcstool \
      python3-colcon-common-extensions \
      mesa-utils \
      git \
      tree \
    && rm -rf /var/lib/apt/lists/*

# AI 関連（CPU 版 PyTorch を先に入れる）
COPY requirements.txt /tmp/requirements.txt
RUN pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu \
 && pip install -r /tmp/requirements.txt

# 作業ユーザー
USER ubuntu
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> ~/.bashrc

WORKDIR /ws
