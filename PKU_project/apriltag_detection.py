import pupil_apriltags as apriltag
import cv2
import rospy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

[fx, fy, cx, cy] = [554.254691191187, 320.5, 554.254691191187, 240.5]  # rostopic camera_info

def color_image_callback(msg):
    K = [fx, fy, cx, cy]
    # print(K)
    bridge = CvBridge()
    try:
        # 将ROS图像消息转换为OpenCV格式
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logerr("Error converting image: %s", str(e))
        return

    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Build a detector for apriltag
    tag_detector = apriltag.Detector()

    # Perform apriltag detection to get a list of detected apriltag
    tags = tag_detector.detect(gray, estimate_tag_pose=True, camera_params=K, tag_size=0.2)

    print("%d apriltags have been detected." % len(tags))

    for tag in tags:
        cv2.circle(cv_image, tuple(tag.corners[0].astype(int)), 4, (0, 0, 255), 2)  # left-top
        cv2.circle(cv_image, tuple(tag.corners[1].astype(int)), 4, (0, 0, 255), 2)  # right-top
        cv2.circle(cv_image, tuple(tag.corners[2].astype(int)), 4, (0, 0, 255), 2)  # right-bottom
        cv2.circle(cv_image, tuple(tag.corners[3].astype(int)), 4, (0, 0, 255), 2)  # left-bottom
        print("family:", tag.tag_family)
        print("id:", tag.tag_id)
        # print("pose_R:", tag.pose_R)
        # print("pose_t:", tag.pose_t)
        # print("pose_err:", tag.pose_err)
        # print("conners:", tag.corners)
        # print("homography:", tag.homography)  # 单应矩阵

        rotation_quaternion = Rotation.from_matrix(tag.pose_R).as_quat()

        pose_message = Pose()
        pose_message.position = Point(x=1.0, y=2.0, z=0.0)
        pose_message.orientation = Quaternion(x=rotation_quaternion[0], y=rotation_quaternion[1],
                                              z=rotation_quaternion[2], w=rotation_quaternion[3])

        # rospy.init_node('Quaternion_detect', anonymous=True)
        pub = rospy.Publisher('Quaternion_detect', Pose, queue_size=10)
        pub.publish(pose_message)

        # # 归一化四元数
        # normalized_quaternion = rotation_quaternion / np.linalg.norm(rotation_quaternion)
        #
        # rotation_matrix = tag.pose_R
        # euler_angles = np.degrees(np.around(np.array([np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2]),
        #                                               np.arctan2(-rotation_matrix[2, 0], np.sqrt(
        #                                                   rotation_matrix[2, 1] ** 2 + rotation_matrix[2, 2] ** 2)),
        #                                               np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])]),
        #                                     decimals=6))
        #
        # # 输出欧拉角
        # print("Euler Angles (in degrees):", euler_angles)
        #
        # print("Quaternion:", rotation_quaternion)
        # print("Normalized Quaternion:", normalized_quaternion)

        cv2.imshow("apriltag_test", cv_image)
        cv2.waitKey(1)  # 保持图像显示，不要太长，否则可能导致响应变慢


if __name__ == '__main__':
    rospy.init_node('depth_subscriber', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw', Image, color_image_callback)

    # 进入ROS循环
    rospy.spin()





