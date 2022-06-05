from PyKDL import Frame, Rotation, Vector
from accel_challenge.challenge2.tool import RPY2T, T2PoseStamped
from accel_challenge.challenge2.examples.calibrate import set_error
import time
import numpy as np
pi = np.pi

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import rospy
from pathlib import Path


move_arm = 'psm2'

#===self testing (comment when evaluation)
joint_calibrate_offset_gt = np.array(np.deg2rad([1,2,0,0,0,0]))
joint_calibrate_offset_gt[2] = 0.02
joint_calibrate_offset_gt = np.array(np.deg2rad([0,0,0,0,0,0]))
# joint_calibrate_offset = np.array([0,0,0,0,0,0])
print("setting error: ", joint_calibrate_offset_gt)
set_error(move_arm, joint_calibrate_offset_gt.tolist())
needle_pub = rospy.Publisher('/CRTK/Needle/servo_cp', PoseStamped)
needle_zero_force_pub = rospy.Publisher('/CRTK/Needle/zero_force', Bool)
rate = rospy.Rate(100)
needle_pos0 = [-0.25786757338201337, 0.5619611862776279, 0.7417253877244148]
needle_pos0 = [0.25786757338201337, 0.5619611862776279, 0.7417253877244148]
needle_pos0 = [-0.25786757338201337, -0.3619611862776279, 0.7417253877244148]
needle_pos0 = [0.25786757338201337, -0.1619611862776279, 0.7417253877244148]



# needle_rpy0 = [0.03031654271074325, 0.029994510295635185, -0.00018838556827461113]
needle_rpy0 = [0.03031654271074325, 0.029994510295635185, np.deg2rad(-150)]
# needle_rpy0 = [0.03031654271074325, 0.029994510295635185, np.deg2rad(-120)]
# needle_rpy0 = [0.03031654271074325, 0.029994510295635185, np.deg2rad(-90)]
needle_rpy0 = [0.03031654271074325, 0.029994510295635185, np.deg2rad(-45)]
needle_rpy0 = [0.03031654271074325, 0.029994510295635185, np.deg2rad(0)]
needle_rpy0 = [0.03031654271074325, 0.029994510295635185, np.deg2rad(45)]
needle_rpy0 = [0.03031654271074325, 0.029994510295635185, np.deg2rad(90)]
needle_rpy0 = [0.03031654271074325, 0.029994510295635185, np.deg2rad(135)]
needle_rpy0 = [0.03031654271074325, 0.029994510295635185, np.deg2rad(170)]
time.sleep(1)
for i in range(2):
    msg = T2PoseStamped(RPY2T(*needle_pos0, *needle_rpy0))
    needle_pub.publish(msg)
    rate.sleep()
time.sleep(2)
for i in range(2):
    msg = Bool()
    msg.data = True
    needle_zero_force_pub.publish(msg)
    rate.sleep()
time.sleep(1)