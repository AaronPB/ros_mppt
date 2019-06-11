#!/usr/bin/env python

"""
    This file is part of ros_mppt.
    ros_mppt is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.
    ros_mppt is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with ros_mppt.  If not, see <https://www.gnu.org/licenses/>.
"""


import rospy
import time
import serial
import logging
from ros_mppt.msg import mppt

logging.basicConfig(level=logging.INFO, filename='ros_mppt.log', filemode='a', format='[%(asctime)s - %(levelname)s]: %(message)s', datefmt='%d-%b-%y %H:%M:%S')
logging.info('Initializing ros_mppt package...')

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

        try:
            ve_read = ser.readline().decode("utf-8")
        except: logging.warning('Skipping decoding from ve_read line: ' + ve_read)

        if "V" in ve_read and "P" not in ve_read:
            try:
                ve_read = ve_read.split("\t")
                msg.v_bat = float(ve_read[1]) * 0.001
            except Exception as e: logging.error('Exception ocurred in V', exc_info=True)
        elif "I" in ve_read and "P" not in ve_read:
            try:
                ve_read = ve_read.split("\t")
                msg.i_bat = float(ve_read[1]) * 0.001
            except Exception as e: logging.error('Exception ocurred in I', exc_info=True)
        elif "VPV" in ve_read:
            try
                ve_read = ve_read.split("\t") * 0.001
                msg.v_pv = float(ve_read[1])
            except Exception as e: logging.error('Exception ocurred in VPV', exc_info=True)
        elif "PPV" in ve_read:
            try:
                ve_read = ve_read.split("\t")
                msg.p_pv = float(ve_read[1])
            except Exception as e: logging.error('Exception ocurred in PPV', exc_info=True)

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
