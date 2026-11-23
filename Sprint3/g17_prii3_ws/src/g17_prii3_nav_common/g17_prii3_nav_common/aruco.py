"""Recibe imágenes de Gazebo y publica los ArUcos detectados."""
import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .vision import detect, detector_aruco


class Aruco(Node):
    def __init__(self):
        super().__init__('aruco_camera')
        param = lambda key, value: self.declare_parameter(key, value).value
        self.show = param('show', False)
        self.topic = param('output_topic', '/g17/aruco_robot')
        self.publisher = self.create_publisher(String, self.topic, 1)
        self.detector, self.bridge = detector_aruco(), CvBridge()
        self.last_received = time.monotonic()
        self.counter = 0
        topic = param('image_topic', '/camera/image_raw')
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self.create_timer(0.2, self.watchdog)

    def publish(self, image=None, state='ok', stamp=None):
        markers = detect(image, self.detector) if image is not None else []
        self.counter += 1
        now = self.get_clock().now().nanoseconds / 1e9
        data = dict(estado=state, unidad='px', frame_stamp=now if stamp is None else stamp,
                    secuencia=self.counter, ancho=0 if image is None else image.shape[1],
                    alto=0 if image is None else image.shape[0], aruco=markers)
        self.publisher.publish(String(data=json.dumps(data)))
        if self.show and image is not None:
            cv2.imshow(self.topic, image)
            cv2.waitKey(1)

    def on_image(self, message):
        self.last_received = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        try:
            self.publish(self.bridge.imgmsg_to_cv2(message, 'bgr8'), stamp=stamp)
        except (CvBridgeError, cv2.error, TypeError, ValueError, RuntimeError) as error:
            self.get_logger().error(str(error))
            self.publish(state='error_imagen')

    def watchdog(self):
        if time.monotonic() - self.last_received > 1.0:
            self.publish(state='sin_imagen')

    def destroy_node(self):
        if self.show:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Aruco()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
