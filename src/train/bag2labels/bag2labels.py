import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import glob
import matplotlib.pyplot as plt
import numpy as np
import os
from PIL import Image
import shutil

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]

def get_ts_messages(parser):
    ts_messages = parser.get_messages("/throttle_steer")
    t = []
    throttle_steer = []
    for message in ts_messages:
        timestamp = message[0]
        throttle= message[1].data[0]
        steer = message[1].data[1]
        t.append(timestamp)
        throttle_steer.append([throttle,steer])
    return np.array(t),np.array(throttle_steer)

def get_camera_messages(parser):
    im_messages = parser.get_messages("/cam/camera/image_raw")
    t = []
    images = []
    for message in im_messages:
        timestamp = message[0]
        w,h,c = 320,180,3 # rosで記録する解像度
        #w,h,c = 180,320,3 # 実機のrosbag画像がなぜか縦横逆になっている
        image = np.array(message[1].data).reshape(h,w,c)
        
        t.append(timestamp)
        images.append(image)
        #pil_img = Image.fromarray(image)
        #pil_img.save('image.jpg')
    return np.array(t), images

def sync_ts_and_images(t_ts, y_ts, t_im, y_im):
    # t_ts: throttle_steerのタイムスタンプ配列(ナノ秒)
    # y_ts: throttle_steerの2次元配列
    # t_im: imageのタイムスタンプ配列(ナノ秒)
    # y_im: image(np.array)の配列

    dt = 0.5 # seconds

    clip_before = 0 # seconds
    clip_after = 0 # seconds
    t_start = max(t_ts[0], t_im[0]) + clip_before*10**9
    t_end = min(t_ts[-1], t_im[-1]) - clip_after*10**9
    print("start: {}".format(t_start/10**9))
    print("end: {}".format(t_end/10**9))
    print("duration: {}".format((t_end-t_start)/10**9))
    
    n = int((t_end-t_start)/10**9 / dt)
    n = max(n-1, 0) # subtract 1, but keep it non-negative
    t = t_start
    synchronized = [] # each row consists of timestamp, throttle_steer, image
    for i in range(n):
        # 時刻がt以上となる最初の位置を計算
        i_ts = np.where(t_ts >= t)[0][0]
        i_im = np.where(t_im >= t)[0][0]

        # その時刻のthrottle_steerとimageをセットとみなす
        ts = y_ts[i_ts]
        im = y_im[i_im]
        synchronized.append([t,ts,im])

        # 時刻を更新
        t += dt*10**9

    return synchronized

if __name__ == "__main__":
    bag_files = glob.glob(os.path.dirname(os.path.abspath(__file__))+"/input_data/**/*.db3")
    #bag_file = bag_files[0]

    # Clean output dir
    output_dir = os.path.dirname(os.path.abspath(__file__))+"/output_data"
    if os.path.exists(output_dir):
    	shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)

    for bag_file in bag_files:
        print(bag_file)
        parser = BagFileParser(bag_file)

        print("Extracting throttle_steer")
        t_ts, y_ts = get_ts_messages(parser)
        print(t_ts)

        print("Extracting images")
        t_im, y_im = get_camera_messages(parser)

        print("Synchronizing data")
        synchronized = sync_ts_and_images(t_ts, y_ts, t_im, y_im)

        print("Writing file")
        for s in synchronized:
            timestamp = s[0]
            throttle = s[1][0]
            steer = s[1][1]
            image = s[2]
            filename = str(int(s[0]))+".jpg"
            # write labels
            with open(output_dir+"/labels.csv", "a") as f:
                # throttle, steerは100で割り-1から1の間に正規化する
                f.write("{},{},{}\n".format(filename,throttle/100,steer/100))
            # write image
            pil_img = Image.fromarray(image)
            pil_img.save(output_dir+"/"+filename)
