import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
    
def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)
    
def main(args=None):
    settings = saveTerminalSettings()

    rclpy.init(args=args)

    node = rclpy.create_node('key_input')
    pub = node.create_publisher(String, 'key', 10)
    
    try:
        while True:
            key = getKey(settings)
            
            # Ctrl-C to exit
            if (key == '\x03'):
                break
            
            msg = String()
            msg.data = key
            pub.publish(msg)
            node.get_logger().info('Publishing: "%s"' % msg.data)
             
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()

