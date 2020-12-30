import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import TwistStamped
import message_filters
from lgsvl_msgs.msg import Detection2DArray
from datetime import datetime
import os
import errno
import numpy as np
import csv
import cv2
from train.utils import mkdir_p, IMG_PATH
import json


class Collect(Node):
    def __init__(self):
        super().__init__('collect', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.get_logger().info('[{}] Initializing...'.format(self.get_name()))
        mkdir_p(IMG_PATH)

        sub_center_camera = message_filters.Subscriber(self, CompressedImage, '/lgsvl/front_6mm_image_compressed')
        ground_truth_2d = message_filters.Subscriber(self, Detection2DArray, '/lgsvl/export_2d_ground_truth_data_topic')

        ts = message_filters.ApproximateTimeSynchronizer([sub_center_camera, ground_truth_2d], 1, 0.1)
        ts.registerCallback(self.callback)

        self.get_logger().info('[{}] New Up and running...'.format(self.get_name()))

    def callback(self, center_camera, ground_truth_2d_data):
        msg_id = str(datetime.now().isoformat())
        detection_result = [];
        for detection in ground_truth_2d_data.detections:
            detection_result.append({
                'label': detection.label,
                'x': detection.bbox.x,
                'y': detection.bbox.y,
                'width': detection.bbox.width,
                'height': detection.bbox.height
            });
        self.save_ground_truth_2d(json.dumps(detection_result), msg_id)
        self.save_image(center_camera, msg_id)
    
    def save_image(self, center_camera, msg_id):
        center_img_np_arr = np.fromstring(bytes(center_camera.data), np.uint8)
        center_img_cv = cv2.imdecode(center_img_np_arr, cv2.IMREAD_COLOR)
        file_path = os.path.join(IMG_PATH, 'center-{}.jpg'.format(msg_id))
        cv2.imwrite(file_path, center_img_cv)

    def save_ground_truth_2d(self, ground_truth_2d_data_json, msg_id):
        file_path = os.path.join(IMG_PATH, 'center-{}.json'.format(msg_id))
        f = open(file_path, 'w')
        f.write(ground_truth_2d_data_json)
        f.close()


def main(args=None):
    rclpy.init(args=args)
    collect = Collect()
    rclpy.spin(collect)


if __name__ == '__main__':
    main()
