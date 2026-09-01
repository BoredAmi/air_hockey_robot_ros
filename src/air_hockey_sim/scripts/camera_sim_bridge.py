#!/usr/bin/env python3
"""Adapts the simulated overhead camera (worlds/air_hockey_table.sdf's
"table_camera" sensor, bridged into ROS as a plain sensor_msgs/Image by
ros_gz_bridge - see the launch file) into the exact message perception_node
actually expects on /camera/raw_stream: a PuckState carrying a JPEG
sensor_msgs/CompressedImage plus a timestamp.

In the real deployment, a Pi-side camera node (not part of this repo)
publishes that PuckState directly - this script exists purely to stand in
for that node when running against the simulator instead of hardware, so
perception_node itself needs zero changes either way.

Usage:
    ros2 run air_hockey_sim camera_sim_bridge.py
    # or directly:
    python3 camera_sim_bridge.py --image-topic /table_camera/image
"""
import argparse
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from air_hockey_robot_msgs.msg import PuckState


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    """Converts a sensor_msgs/Image (as bridged from gz's R8G8B8 camera
    format, typically arriving as ROS encoding 'rgb8') into an OpenCV BGR
    array. Assumes no row padding (step == width * channel_count), true for
    ros_gz_bridge's output."""
    if msg.encoding in ("rgb8", "bgr8"):
        channels = 3
    elif msg.encoding == "mono8":
        channels = 1
    else:
        raise ValueError(f"unsupported image encoding from sim camera: {msg.encoding}")

    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if channels == 1:
        arr = arr.reshape(msg.height, msg.width)
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)

    arr = arr.reshape(msg.height, msg.width, channels)
    if msg.encoding == "rgb8":
        arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    return arr


class CameraSimBridge(Node):
    def __init__(self, image_topic: str, jpeg_quality: int):
        super().__init__("camera_sim_bridge")
        self.jpeg_quality = jpeg_quality
        self.frame_count = 0

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(PuckState, "/camera/raw_stream", pub_qos)
        self.sub = self.create_subscription(Image, image_topic, self.on_image, sub_qos)
        self.get_logger().info(
            f"Bridging {image_topic} (sensor_msgs/Image) -> /camera/raw_stream (PuckState), "
            f"jpeg_quality={jpeg_quality}"
        )

    def on_image(self, msg: Image):
        try:
            bgr = image_msg_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return

        ok, jpeg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            self.get_logger().error("JPEG encode failed", throttle_duration_sec=5.0)
            return

        out = PuckState()
        # Microseconds, matching the convention every other timestamp in
        # this pipeline uses (see PuckDetection/PredictedEntry) - this
        # script is standing in for the Pi's own camera-capture timestamp.
        out.timestamp = int(time.time() * 1e6)
        out.is_detected = False  # unused by perception_node's image_callback
        out.image_frame.format = "jpeg"
        out.image_frame.data = jpeg.tobytes()
        self.pub.publish(out)

        self.frame_count += 1
        if self.frame_count % 90 == 0:
            self.get_logger().info(f"{self.frame_count} frames bridged so far", throttle_duration_sec=5.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--image-topic", default="/table_camera/image",
                         help="ROS-bridged sensor_msgs/Image topic to read from")
    parser.add_argument("--jpeg-quality", type=int, default=90)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = CameraSimBridge(args.image_topic, args.jpeg_quality)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
