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
    """
    Convert a quaternion to roll, pitch, and yaw angles (in radians).
    """
    x, y, z, w = q.x, q.y, q.z, q.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('eye_node')

        # Set up TF listener for coordinate frame transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to RGB image topic
        self.subscription_image = self.create_subscription(
            Image,
            '/camera_depth/image_raw',
            self.image_callback,
            10)

        # Subscribe to depth image topic
        self.subscription_dimage = self.create_subscription(
            Image,
            '/camera_depth/depth/image_raw',
            self.dimage_callback,
            10)

        # Subscribe to camera calibration info for depth camera intrinsics
        self.subscription_int = self.create_subscription(
            CameraInfo,
            '/camera_depth/camera_info',
            self.ins_callback,
            10)

        # Subscribe to odometry for robot pose data
        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        # Initialize variables to store incoming messages
        self.ins = None  # camera info
        self.image = None  # current RGB image
        self.dimage = None  # current depth image

        self.br = CvBridge()  # bridge to convert ROS images to OpenCV format

        # Timer to trigger processing at 5 Hz (every 0.2 seconds)
        self.timer = self.create_timer(0.2, self.timer_callback)

    def odom_callback(self, msg):
        # Extract and convert robot orientation quaternion to roll, pitch, yaw
        self.location = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        _, _, self.orientation = quaternion_to_rpy(msg.pose.pose.orientation)

    def ins_callback(self, data):
        # Store camera calibration info for later depth processing
        self.ins = data

    def tf_from_cam_to_map(self):
        # Lookup transform from camera frame to map frame for point conversion
        from_frame = 'camera_rgb_optical_frame'
        to_frame = 'map'
        now = rclpy.time.Time()

        try:
            tf = self.tf_buffer.lookup_transform(
                to_frame, from_frame, now, timeout=rclpy.duration.Duration(seconds=1.0))
            return tf
        except:
            return None

    def image_callback(self, data):
        # Convert ROS Image message to OpenCV BGR format and store
        self.image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

    def dimage_callback(self, data):
        # Convert ROS Image message to OpenCV depth format and store
        self.dimage = self.br.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def timer_callback(self):
        # Ensure we have all needed data before processing
        if self.image is None or self.dimage is None or self.ins is None:
            return

        current_frame = self.image

        # Convert the image from BGR to HSV color space for better color filtering
        hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # Define the HSV range for detecting green color (adjustable)
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])

        # Create a binary mask where green colors within the range are white
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Bitwise-AND to keep only green parts of the image
        res = cv2.bitwise_and(current_frame, current_frame, mask=mask)

        # Find contours (boundaries) of white regions in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        centroids = []

        for cnt in contours:
            area = cv2.contourArea(cnt)

            # Ignore small contours that are likely noise
            if area > 200:
                M = cv2.moments(cnt)

                # Calculate centroid coordinates if contour area is non-zero
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    centroids.append((cx, cy))

                    # Draw a circle on centroid for visualization
                    cv2.circle(res, (cx, cy), 7, (0, 255, 255), -1)

        # If no centroids found, just display and wait
        if not centroids:
            cv2.imshow("Detected Green Objects", res)
            cv2.waitKey(1)
            return

        # Extract depth value for each centroid pixel from depth image
        depths = []
        for c in centroids:
            x, y = c
            depths.append(self.dimage[y, x])

        # Prepare camera intrinsics for depth to 3D point conversion
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

        # Convert pixel coordinates and depth to real-world 3D points
        points_3d = [rs.rs2_deproject_pixel_to_point(_intrinsics, centroids[i], depths[i]) for i in range(len(centroids))]

        # Transform and log points in the map coordinate frame
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

        # Display the result window with green areas and centroids marked
        cv2.imshow("Detected Green Objects", res)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
