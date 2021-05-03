# Racer

自動運転ミニカー

## 事前準備

Ubuntu 20.04

ros2, colcon, gazeboをインストールしておく。

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

Nodes:
* keyboard: キーボード入力の受付
* key_ctl: キーボード入力をスロットルとステアのコマンドに変換
* s_motor: スロットルとステアのコマンドをGazeboが受け付ける型に変換

Topics:
* key: std_msgs/String キー入力
* throttle_steer: std_msgs/Int8MultiArray スロットル/ステアコマンド(+-100の整数)
* /demo/cmd_demo: geometry_msgs/Twist 速度/角速度コマンド

keyboard -> key_ctl -> s_motor -> gazebo


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


## 参考

ROS2からGazeboを起動する方法

https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/

https://zmk5.github.io/general/demo/2019/07/15/ros2-spawning-entity.html


