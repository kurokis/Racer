import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import glob
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


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
        w,h,c = 160,120,3
        image = np.array(message[1].data).reshape(h,w,c)
        t.append(timestamp)
        images.append(image)
        #pil_img = Image.fromarray(image)
        #pil_img.save('image.jpg')
    return np.array(t), images

def sync_ts_and_images(t_ts, y_ts, t_im, y_im):
    # t_ts: throttle_steerのタイムスタンプ配列
    # y_ts: throttle_steerの2次元配列
    # t_im: imageのタイムスタンプ配列
    # y_im: image(np.array)の配列

    sample_rate = 10 # Hz

    # TODO: implement syncing
    #t = t_ts[0]
    #ts_indices = []
    #for i in range(len(t_ts)):
    #    if (t_ts[i]/10**9)>(t/10**9)+1.0/sample_rate:
    #        # update timestamp on the ts side
    #        t = t_ts[i]
    #        ts_indices.append(i)
    #print(ts_indices)
        
if __name__ == "__main__":

    #bag_file = '/workspaces/foxy_leg_ws/rosbag2_2021_04_28-04_42_07/rosbag2_2021_04_28-04_42_07_0.db3'
    bag_files = glob.glob("input_data/**/*.db3")
    bag_file = bag_files[0]
    print(bag_file)
    parser = BagFileParser(bag_file)

    t_ts, y_ts = get_ts_messages(parser)
    print(t_ts)

    t_im, y_im = get_camera_messages(parser)

    sync_ts_and_images(t_ts, y_ts, t_im, y_im)