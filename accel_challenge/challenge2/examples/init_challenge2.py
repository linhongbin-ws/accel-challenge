from accel_challenge.challenge2.tool import RPY2T, T2PoseStamped
from accel_challenge.challenge2.examples.calibrate import set_error
import time
import numpy as np
pi = np.pi

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import rospy
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('-s', required=True, type=int) # program type, 1 for recording, 2 for trajectory
args, remaining = parser.parse_known_args()

move_arm = 'psm2'

#===self testing (comment when evaluation)
# joint_calibrate_offset_gt = np.array(np.deg2rad([1,2,0,0,0,0]))
# joint_calibrate_offset_gt[2] = 0.02
# joint_calibrate_offset_gt = np.array(np.deg2rad([0,0,0,0,0,0]))
# # joint_calibrate_offset = np.array([0,0,0,0,0,0])
# print("setting error: ", joint_calibrate_offset_gt)
# set_error(move_arm, joint_calibrate_offset_gt.tolist())


rng_error = np.random.RandomState(11) # use local seed
ERROR_MAG_ARR = np.deg2rad([5,5,0, 0,0,0])
ERROR_MAG_ARR[2] = 0.05 # simulation unit, for insertion joint
for i in range(args.s):
    _error = np.random.uniform(-ERROR_MAG_ARR,ERROR_MAG_ARR)
set_error('psm2',_error)


needle_pub = rospy.Publisher('/CRTK/Needle/servo_cp', PoseStamped, queue_size=1)
needle_zero_force_pub = rospy.Publisher('/CRTK/Needle/zero_force', Bool,queue_size=1)
rate = rospy.Rate(100)
needle_pos0 = [-0.25786757338201337, 0.5619611862776279, 0.7417253877244148]
needle_pos0 = [0.25786757338201337, 0.5619611862776279, 0.7417253877244148]
needle_pos0 = [-0.25786757338201337, -0.3619611862776279, 0.7417253877244148]
needle_pos0 = [0.25786757338201337, -0.1619611862776279, 0.7417253877244148]
low = np.array([-0.25, -0.36, 0.75])
high = np.array([0.25, 0.56, 0.75])
for i in range(args.s):
    needle_pos0 = np.random.uniform(low, high).tolist()


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
for i in range(args.s):
    needle_rpy0 = [0.03031654271074325, 0.029994510295635185, np.random.uniform(-pi, pi)]
time.sleep(1)
print("needle pos", needle_pos0)
print("needle rpy", needle_rpy0)
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

