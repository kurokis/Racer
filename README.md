# Racer



## 事前準備

Ubuntu 20.04

ros2, colcon, gazeboをインストールしておく。


## ビルド

リポジトリのルート(README.mdがある場所)に移動

```bash
cd ros_ws
```

colconでracerパッケージをビルド

```bash
colcon build --packages-select racer
```

setup.bashをソースする（installフォルダはビルド後に作成される）

```bash
. install/setup.bash
```

launchファイルを使って必要なノードやgazeboをまとめて起動する

```bash
ros2 launch racer gazebo_manual.launch.py
```


## 参考

ROS2からGazeboを起動する方法

https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/

https://zmk5.github.io/general/demo/2019/07/15/ros2-spawning-entity.html


