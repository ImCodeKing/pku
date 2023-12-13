import rospy
from geometry_msgs.msg import Twist

cmd_vel_pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
#初始化节点
rospy.init_node('wander')
twist=Twist()
rate=rospy.Rate(10)

while not rospy.is_shutdown():
    twist.linear.x = 0.5
    cmd_vel_pub.publish(twist)
    rate.sleep()