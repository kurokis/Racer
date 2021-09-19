# 使い方

前提: nvidia-container-runtimeがインストールされており、dockerコンテナ内でGPUが利用できる状態であること

1. ```bash docker_build.sh```でTensorRTを実行するためのコンテナを作成する

2. input_dataに最適化前のモデルをmodel.ptという名前で格納する

3. ```bash docker_run.sh```を実行するとoutput_dataに最適化後のモデルmodel_trt.ptが作成される