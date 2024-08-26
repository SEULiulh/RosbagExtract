#! /usr/bin/env python2

import rospy                    # Library to use Python
# import numpy as np
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Header    # Library to use IMU
# from nav_msgs .msg import Odometry

############################################### Name of the Imu ######
imuData_pub = rospy.Publisher("/imu/new_data", Imu, queue_size=10)

############################################### Tranformation from Euler to Quat ######
# def euler_to_quat(yaw, pitch, roll):
#     q0 = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#     q1 = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
#     q2 = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
#     q3 = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#     return [q0, q1, q2, q3]

############################################## Read the Sensor
def imuCallback(odom_msg):

    Fabio_Imu = Imu()
############################################ Start Header Msgs
    Fabio_Imu.header.seq = 0            # Growing Var   
    Fabio_Imu.header.stamp = odom_msg.header.stamp   # Timestamp
    Fabio_Imu.header.frame_id = "Fabio_TF"  # Name of Frame

    Fabio_Imu.orientation.x = odom_msg.orientation.x           # Quat Data in orient. Lib
    Fabio_Imu.orientation.y = odom_msg.orientation.y    
    Fabio_Imu.orientation.z = odom_msg.orientation.z    
    Fabio_Imu.orientation.w = odom_msg.orientation.w    

    #Fabio_Imu.angular_velocity.x = Quaternion[0]
    #Fabio_Imu.angular_velocity.y = Quaternion[0]
    #Fabio_Imu.angular_velocity.z = Quaternion[0]

    #Fabio_Imu.linear_acceleration.x = Quaternion[0]
    #Fabio_Imu.linear_acceleration.y = Quaternion[0]
    #Fabio_Imu.linear_acceleration.z = Quaternion[0]

    imuData_pub.publish(Fabio_Imu)      # Publish the Node
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = Fabio_Imu.header.stamp
    t.header.frame_id = "world"
    t.child_frame_id = "Fabio_TF"

    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0

    t.transform.rotation = Fabio_Imu.orientation
    br.sendTransform(t)

########################################################
############################## Main ####################
########################################################

if __name__ == '__main__':
    rospy.init_node('test_1_IMU')               # Name of Init_Node
    rospy.loginfo('Show imu data in rviz')         # just print command
    rospy.Subscriber("/imu/data", Imu, imuCallback)
    rospy.spin()