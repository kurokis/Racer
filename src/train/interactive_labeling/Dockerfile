FROM python:3.9

WORKDIR /app

# opencv-devのインストール
RUN apt-get update -y && apt-get install -y libopencv-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install numpy pandas matplotlib opencv-python ipywidgets jupyterlab

RUN pip3 install ipympl

CMD ["jupyter","lab","--port=8888","--no-browser","--ip=0.0.0.0","--allow-root"]

