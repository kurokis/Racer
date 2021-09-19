# このファイルの親フォルダにカレントディレクトリを移動
cd "$(dirname "$0")"
cd ..

#Docker実行:
# - 実行するコンテナの名前を指定(--name)
# - 終了時にイメージ削除(--rm)
# - インタラクティブセッションで実行(-it)
# - リポジトリのルートを/appにbind mount(--mount)
# - 共有メモリサイズ512MB(必要に応じ調整)(--shm-size)
# - ホストPCにGUIを表示させる(--net, -e, -v, 事前のxhost +実行)
# - ジョイスティック対応(--privileged, --device)
# - CSIカメラ対応(--device, --ipc, -v /tmp/argus_socket)
# - イメージ名hgpt2/racer-jetson(docker_build.shで指定済み)
sudo xhost +si:localuser:root
sudo docker run --name racer --rm -it --privileged \
    --mount type=bind,source="$(pwd)",target=/app \
    --runtime nvidia \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    --device=/dev/input/js0:/dev/input/js0 \
    --device=/dev/snd:/dev/snd \
    --device=/dev/video0:/dev/video0 \
    --ipc=host \
    -v /tmp/argus_socket:/tmp/argus_socket \
    hgpt2/racer-jetson

# test code to check whether camera is recognized from inside the container:
# gst-launch-1.0 nvarguscamerasrc ! 'video/x-raw(memory:NVMM),width=3820, height=2464, framerate=21/1, format=NV12' ! nvvidconv flip-method=0 ! 'video/x-raw,width=960, height=616' ! nvvidconv ! nvegltransform ! nveglglessink -e