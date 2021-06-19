# デフォルトのベースイメージはJetson向けにl4t-mlとするが、
# PC用のイメージをビルドする場合は別のベースイメージをBASE_IMAGEで指定する
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-ml:r32.5.0-py3
FROM $BASE_IMAGE
USER root

# ロケールを日本語に変更
RUN apt-get update
RUN apt-get -y install locales && \
    localedef -f UTF-8 -i ja_JP ja_JP.UTF-8
ENV LANG ja_JP.UTF-8
ENV LANGUAGE ja_JP:ja
ENV LC_ALL ja_JP.UTF-8
ENV TZ JST-9

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
