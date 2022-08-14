#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist


def adding_noise(data):
    noise_data = Twist()
    noise = np.random.normal(0, 1, (2, 3))
    noise_data.linear.x = data.linear.x + noise[0][0]
    noise_data.linear.y = data.linear.y + noise[0][1]
    noise_data.linear.z = data.linear.z + noise[0][2]
    noise_data.angular.x = data.angular.x + noise[1][0]
    noise_data.angular.y = data.angular.y + noise[1][1]
    noise_data.angular.z = data.angular.z + noise[1][2]
    pub.publish(noise_data)


if __name__ == "__main__":
    try:
        rospy.init_node("vel_noise_adder", anonymous=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pub = rospy.Publisher("noised_vel", Twist, queue_size=1000)
            rospy.Subscriber("cmd_vel", Twist, adding_noise)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
