import rospy
from geometry_msgs.msg import Pose, Point, Quaternion


def Quaternion_detect_callback(msg):
    print(msg.orientation)
    print('--------------------------------')


if __name__ == '__main__':
    rospy.init_node('depth_subscriber', anonymous=True)
    rospy.Subscriber('/Quaternion_detect', Pose, Quaternion_detect_callback)

    # 进入ROS循环
    rospy.spin()