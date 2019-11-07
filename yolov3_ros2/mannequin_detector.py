import argparse
import time
import json
import rclpy
#from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
# from std_msgs.msg import String
from sensor_msgs.msg import TimeReference
import cv2
from cv_bridge import CvBridge
from yolo import YOLO
from PIL import Image as PIL_IMAGE
from utils import generate_colors, make_r_image

rclpy.init()
node = rclpy.create_node("mannequin_detector")
bridge = CvBridge()
image = None
header = Header()

def main(args):
    model = YOLO(**vars(args))
    
    node.create_subscription(Image, 'usb_cam/image_raw', callback)
    pub = node.create_publisher(TimeReference, 'people_info')
    msg = TimeReference()

    while rclpy.ok():
    # for _ in range(5):
        rclpy.spin_once(node)
        try:
            result = detect_image(model, args.classes_path)
            print(result)
        except:
            result = {'objects' : []}
            print('no people')
        msg.header = header
        msg.source = json.dumps(result)
        pub.publish(msg)

    model.close_session()
    node.destroy_node()
    rclpy.shutdown()

def callback(msg):
    global image
    global header
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, 'rgb8')
        header = msg.header
    except CvBridgeError as e:
        print(e)
    
    # cv2.imshow('image', cv_image)
    # cv2.waitKey(3)
    image = cv2pil(cv_image)

def cv2pil(cv_image):
    new_image = cv_image.copy()
    if new_image.ndim == 2:
        pass
    elif new_image.shape[2] == 3:
        new_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    elif new_image.shape[2] == 4:
        new_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2RGBA)
    new_image = PIL_IMAGE.fromarray(new_image)
    return new_image


def detect_image(model, classes_path):
    class_num = 0
    with open(classes_path) as f:
        class_num = len(f.readlines())
    result = model.detect_image(image)
    # colors = generate_colors(class_num)
    # r_image = make_r_image(image, result['objects'], colors)
    # r_image.show()

    return result

    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '-m', '--model_path', type=str, default='./model_data/only_person_coco/model_person.h5',
        help='path to model weight file'
    )

    parser.add_argument(
        '-a', '--anchor_path', type=str, default='./model_data/only_person_coco/anchor.txt',
        help='path to anchor definitions'
    )

    parser.add_argument(
        '-c', '--classes_path', type=str, default='./model_data/only_person_coco/classes.txt',
        help='path to class definitions'
    )

    args = parser.parse_args()

    main(args)