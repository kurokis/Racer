# Racer

自動運転ミニカー

## 事前準備

ros2, colcon, gazeboをインストールしておく。

Jetson Nano JetPack 4.5.1

 - Ubuntu 18.04
 - ROS2 Eloquent 

[How to install ROS2 Elquent](https://docs.ros.org/en/eloquent/Installation/Linux-Install-Debians.html)

[How to install colcon](https://colcon.readthedocs.io/en/released/user/installation.html)

[How to install Gazebo](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)


## パッケージ構成

```
src/
  racer/
    launch/ launchファイルの格納場所
    models/ 車両モデル（SDF、メッシュ）
    racer/ コード本体
    resource/ 略
    test/ 略
    worlds/ 車両の力学モデル+ワールド
    package.xml パッケージ概要、依存ライブラリを記述
    setup.cfg 略
    setup.py ビルド設定を記述
```

## ソフト構成

![](docs/rqt_graph.png)

Nodes:
* keyboard: キーボード入力の受付
* key_ctl: キーボード入力をスロットルとステアのコマンドに変換
* joy_ctl: ジョイスティック入力をスロットルとステアのコマンドに変換
* nn_ctl: 画像入力をスロットルとステアのコマンドに変換
* s_motor: スロットルとステアのコマンドをGazeboが受け付ける型に変換
* gzserver: gazebo simulation
* rviz: 可視化

Topics:
* key: std_msgs/String キー入力
* throttle_steer: std_msgs/Int8MultiArray スロットル/ステアコマンド(+-100の整数)
* /demo/cmd_demo: geometry_msgs/Twist 速度/角速度コマンド
* /cam/camera/image_raw: sensor_msgs/Image ROS画像

## ビルドと起動

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

## サンプル画像

![](docs/camera_image.png)

## Windows上で実行する方法

setup_for_windowsフォルダに[docker-ros2-desktop-vnc](https://github.com/Tiryoh/docker-ros2-desktop-vnc)をベースにしたDockerfileを置いている。これを使うとWindowsで簡単にROS2実行環境が構築できる。このコンテナを実行するとROS2、Gazebo、VNC、Visual Studio Code (ターミナルから`code`で実行)、python、pip、pythonライブラリ(pygame等)がインストールされた状態でスタートする。

1. Docker for Windowsをインストールする

1. docker_build.batを実行(初回は5GB程度のファイルをダウンロードするため時間がかかる)

1. docker_run.batを実行

1. Webブラウザを起動し、http://127.0.0.1:6080/ (またはlocalhost:6080)にアクセスする

1. VNC画面が現れることを確認する

![](docs/vnc_screen.jpg)

注意：同じDockerイメージがすでに実行中だと、ポートが競合するためdocker_run.batの実行が失敗する。

## 参考

ROS2からGazeboを起動する方法

https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/

https://zmk5.github.io/general/demo/2019/07/15/ros2-spawning-entity.html


