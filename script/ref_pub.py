#!/usr/bin/env python
PKG = 'tocabi_cc'
import roslib; roslib.load_manifest(PKG)

import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32MultiArray

import numpy

def talker():
    rospy.init_node('rl_motion_generator', anonymous=True)
    pub = rospy.Publisher('rl_motion_reference', Float32MultiArray, queue_size=10)
    r = rospy.Rate(50)

    ref_data = numpy.loadtxt('../motion/processed_data_tocabi_slow_vive_contact_with_force_ref.txt')
    ref_data_len = len(ref_data)

    num_cur_ref_joint = 12
    data_idx = 0
    
    a = Float32MultiArray()

    while not rospy.is_shutdown():
        a.data = numpy.concatenate([ref_data[data_idx,1:1+num_cur_ref_joint],[ref_data[data_idx,36] / 1.5]]).tolist()

        if (data_idx < ref_data_len-1):
            data_idx += 1
        pub.publish(a)
        r.sleep()

if __name__ == '__main__':
    talker()