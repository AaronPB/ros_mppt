#!/usr/bin/env python

import rospy
import time
import serial
from ros_mppt.msg import mppt

def sender():
    pub = rospy.Publisher('mppt_channel', mppt, queue_size=10)
    rospy.init_node('ros_mppt', anonymous=True)
    r = rospy.Rate(10)
    msg = mppt()

    #Predefined values of msg
    msg.v_bat = -1
    msg.i_bat = -1
    msg.v_pv = -1
    msg.p_pv = -1

    while not rospy.is_shutdown():

        ve_read = ser.readline().decode("utf-8")

        if "V" in ve_read and "P" not in ve_read:
            ve_read = ve_read.split("\t")
            msg.v_bat = float(ve_read[1]) * 0.001
        elif "I" in ve_read and "P" not in ve_read:
            ve_read = ve_read.split("\t")
            msg.i_bat = float(ve_read[1]) * 0.001
        elif "VPV" in ve_read:
            ve_read = ve_read.split("\t")
            msg.v_pv = float(ve_read[1])
        elif "PPV" in ve_read:
            ve_read = ve_read.split("\t")
            msg.p_pv = float(ve_read[1])

        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=10)
        sender()
    except rospy.ROSInterruptException: pass
    finally:
        ser.close()
