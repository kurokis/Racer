# デフォルトのベースイメージはJetson向けにl4t-mlとするが、
# PC用のイメージをビルドする場合は別のベースイメージをBASE_IMAGEで指定する
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-ml:r32.5.0-py3
FROM $BASE_IMAGE
USER root

# ロケールを日本語に変更
RUN apt-get update
RUN apt update
RUN apt install locales
RUN apt-get -y install locales && \
    localedef -f UTF-8 -i ja_JP ja_JP.UTF-8
ENV LANG ja_JP.UTF-8
ENV LANGUAGE ja_JP:ja
ENV LC_ALL ja_JP.UTF-8
ENV TZ JST-9

# ROS2
# [How to install ROS2 Eloquent](https://docs.ros.org/en/eloquent/Installation/Linux-Install-Debians.html)
RUN apt update && apt install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add 
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN apt update
RUN apt install -y ros-eloquent-desktop
# shではなくbashを使いたいときは/bin/bashで実行する
RUN ["/bin/bash", "-c", "source /opt/ros/eloquent/setup.bash"]
RUN python3 -m pip install -U argcomplete

# colcon
# [How to install colcon](https://colcon.readthedocs.io/en/released/user/installation.html)
RUN sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update
RUN apt install -y python3-colcon-common-extensions

# Gazebo
# [How to install Gazebo](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
RUN apt install -y ros-eloquent-gazebo-ros-pkgs

# Jetson Nanoでpygameをインストールするのに必要なライブラリ郡
RUN apt update
RUN apt install -y libsdl-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev
RUN apt install -y libsmpeg-dev libportmidi-dev libavformat-dev libswscale-dev
RUN apt install -y libfreetype6-dev
RUN apt install -y libportmidi-dev

# Jetson Nanoに対してはpygame 1.9.6でないとインストールできない報告あり
# https://qiita.com/karaage0703/items/5d43309bc688858e7c2a
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install -U pygame==1.9.6 --user

# PC環境用のOpenCV,Tensorflowインストール
# opencv-devのインストール
#RUN apt-get update -y && apt-get install -y libopencv-dev && apt-get clean && rm -rf /var/lib/apt/lists/*
# TensorflowとOpencvのインストール
#RUN python3 -m pip install numpy tensorflow opencv-python
