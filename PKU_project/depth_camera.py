import rospy
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2

def color_image_callback(msg):
    bridge = CvBridge()
    try:
        # 将ROS图像消息转换为OpenCV格式
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logerr("Error converting image: %s", str(e))
        return

    # 在这里执行您的颜色相机图像处理逻辑，例如显示图像
    cv2.imshow("Color Image", cv_image)
    cv2.waitKey(1)  # 保持图像显示，不要太长，否则可能导致响应变慢

def depth_points_callback(msg):
    # 处理/camera/depth/points话题的回调函数
    gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

    # 处理每个点
    for point in gen:
        x, y, z = point
        # 在这里执行您的处理逻辑，例如打印每个点的坐标
        print(f"Point: ({x}, {y}, {z})")
    print("-----------------------------------")

def depth_image_callback(msg):
    # 处理/camera/depth/image_raw话题的回调函数
    bridge = CvBridge()
    # 将ROS图像消息转换为OpenCV格式
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception as e:
        rospy.logerr("Error converting image: %s", str(e))
        return

    # 显示图像
    cv2.imshow("Depth Image", cv_image)
    cv2.waitKey(1)  # 这个延时是为了确保图像正常显示，不要太长，否则可能导致响应变慢


def main():
    rospy.init_node('depth_subscriber', anonymous=True)

    # 订阅/camera/depth/points话题，注册回调函数depth_points_callback
    rospy.Subscriber('/camera/depth/points', PointCloud2, depth_points_callback)

    # 订阅/camera/depth/image_raw话题，注册回调函数depth_image_callback
    # rospy.Subscriber('/camera/depth/image_raw', Image, depth_image_callback)

    # 订阅颜色相机图像话题，注册回调函数color_image_callback
    rospy.Subscriber('/camera/color/image_raw', Image, color_image_callback)

    # 进入ROS循环
    rospy.spin()

if __name__ == '__main__':
    main()