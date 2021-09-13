import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

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



if __name__ == "__main__":

        bag_file = 'rosbag_sample.db3'

        parser = BagFileParser(bag_file)

        front_camera_image = parser.get_messages("/front_camera_image")
        throttle_steer = parser.get_messages("/throttle_steer")
        with open('throttle_steer.txt', mode='w') as f:
            for i in range(len(throttle_steer)):
                f.write(str(throttle_steer[i][0])+","+str(throttle_steer[i][1])+'\n')
        print(throttle_steer)
        bridge = CvBridge()
        with open('image_timestamp.txt', mode='w') as f:
            for i in range(len(front_camera_image)):
                f.write(str(front_camera_image[i][0])+'\n')
                cv_img=bridge.imgmsg_to_cv2(front_camera_image[i][1])
                cv2.imwrite("tools/image/"+str(front_camera_image[i][0])+".png",cv_img)
