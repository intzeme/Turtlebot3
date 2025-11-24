import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, PointStamped
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point
import numpy as np
import cv2
import pyrealsense2 as rs

def quaternion_to_rpy(q: Quaternion):
    x, y, z, w = q.x, q.y, q.z, q.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('eye_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription_image = self.create_subscription(
            Image,
            '/camera_depth/image_raw',
            self.image_callback,
            10)

        self.subscription_dimage = self.create_subscription(
            Image,
            '/camera_depth/depth/image_raw',
            self.dimage_callback,
            10)

        self.subscription_int = self.create_subscription(
            CameraInfo,
            '/camera_depth/camera_info',
            self.ins_callback,
            10)

        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        self.ins = None
        self.image = None
        self.dimage = None

        self.br = CvBridge()

        self.timer = self.create_timer(0.2, self.timer_callback)

    def odom_callback(self, msg):
        self.location = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        _, _, self.orientation = quaternion_to_rpy(msg.pose.pose.orientation)

    def ins_callback(self, data):
        self.ins = data

    def tf_from_cam_to_map(self):
        from_frame = 'camera_rgb_optical_frame'
        to_frame = 'map'

        now = rclpy.time.Time()

        try:
            tf = self.tf_buffer.lookup_transform(to_frame, from_frame, now, timeout=rclpy.duration.Duration(seconds=1.0))
            return tf
        except:
            return None

    def image_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.image = current_frame

    def dimage_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='passthrough')
        self.dimage = current_frame

    def timer_callback(self):
        if self.image is None or self.dimage is None or self.ins is None:
            return

        current_frame = self.image
        hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # Define green color range in HSV
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)
        res = cv2.bitwise_and(current_frame, current_frame, mask=mask)

        # Find contours for green areas
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 200:  # filter small artifacts
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    centroids.append((cx, cy))
                    cv2.circle(res, (cx, cy), 7, (0, 255, 255), -1)

        # Depth values at centroid pixels
        depths = []
        for c in centroids:
            x, y = c
            depths.append(self.dimage[y, x])

        if not centroids:
            cv2.imshow("camera", res)
            cv2.waitKey(1)
            return

        # From CameraInfo get intrinsics
        cameraInfo = self.ins
        _intrinsics = rs.intrinsics()
        _intrinsics.width = cameraInfo.width
        _intrinsics.height = cameraInfo.height
        _intrinsics.ppx = cameraInfo.k[2]
        _intrinsics.ppy = cameraInfo.k[5]
        _intrinsics.fx = cameraInfo.k[0]
        _intrinsics.fy = cameraInfo.k[4]
        _intrinsics.model = rs.distortion.none
        _intrinsics.coeffs = [i for i in cameraInfo.d]

        points_3d = [rs.rs2_deproject_pixel_to_point(_intrinsics, centroids[i], depths[i]) for i in range(len(centroids))]

        for pt3d in points_3d:
            point = PointStamped()
            point.header.frame_id = 'map'
            point.point.x = pt3d[0]
            point.point.y = pt3d[1]
            point.point.z = pt3d[2]

            tf = self.tf_from_cam_to_map()
            if tf is None:
                continue
            point_world = do_transform_point(point, tf)
            self.get_logger().info(f"Green object at world coords: ({point_world.point.x:.2f}, {point_world.point.y:.2f})")

        cv2.imshow("camera", res)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
