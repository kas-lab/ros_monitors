FROM ros:humble-ros-core-jammy

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    sudo \
    curl \
    wget \
    vim \
    git \
    python3-pip \
    python3-vcstool \
    python3-rosdep \
    htop \
    ros-dev-tools \
    software-properties-common \
    apt-transport-https \
    gpg \
    flake8 \
    python3-flake8-* \
    python3-ament-flake8 \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init

## Create ubuntu-user user, disable password, and add it to sudo group
RUN groupadd -g 1000 ubuntu-user \
    && adduser --disabled-password --gid 1000 --uid 1000 --gecos '' ubuntu-user \
    && adduser ubuntu-user sudo

RUN echo 'ubuntu-user ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers
ENV HOME=/home/ubuntu-user
USER ubuntu-user
ENV DEBIAN_FRONTEND=noninteractive

## Create, install deps, and build ros_typedb workspace
RUN mkdir -p $HOME/ros_monitors_ws/src
COPY . $HOME/ros_monitors_ws/src/ros_monitors

WORKDIR $HOME/ros_monitors_ws
RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
    && sudo apt update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && sudo rm -rf /var/lib/apt/lists/"]

## There is a bug with the default setuptools and packaging versions
RUN USER=ubuntu-user python3 -m pip install setuptools==75.8.2  packaging==24.1 empy==3.3.4 pandas==2.0.2 scipy==1.15.2 numpy==1.23.5
RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install \
    && echo 'source ~/ros_monitors_ws/install/setup.bash' >> ~/.bashrc"]

RUN sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/