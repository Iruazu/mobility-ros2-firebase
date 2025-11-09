FROM osrf/ros:humble-desktop-full

# 基本パッケージとツールのインストール
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    wget \
    curl \
    nano \
    vim \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Navigation2とTurtleBot3関連パッケージ
RUN apt-get update && apt-get install -y \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-gazebo \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# Python依存関係
RUN pip3 install \
    firebase-admin \
    google-cloud-firestore \
    python-dotenv

# 作業ディレクトリ設定
WORKDIR /workspace

# ROS2環境の自動ソース
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /workspace/install/setup.bash 2>/dev/null || true" >> /root/.bashrc
RUN echo "export TURTLEBOT3_MODEL=waffle" >> /root/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspace/src" >> /root/.bashrc

# rosdep初期化
RUN rosdep init || true
RUN rosdep update

# 非rootユーザー作成（VSCode用）
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /workspace

# ユーザーの.bashrc設定
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /workspace/install/setup.bash 2>/dev/null || true" >> /home/$USERNAME/.bashrc
RUN echo "export TURTLEBOT3_MODEL=waffle" >> /home/$USERNAME/.bashrc