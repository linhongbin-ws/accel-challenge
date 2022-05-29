from accel_challenge.challenge2.ros_client import ClientEngine
from PyKDL import Frame, Rotation, Vector
from accel_challenge.challenge2.tool import RPY2T, T2PoseStamped
from accel_challenge.challenge2.param import T_gt_n, T_hover_gt, NEEDLE_R, T_tip_n
from accel_challenge.challenge2.examples.calibrate import set_error, calibrate_joint_error
import time
import numpy as np
pi = np.pi

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import rospy
from surgical_robotics_challenge.task_completion_report import TaskCompletionReport
from pathlib import Path

#===params
team_name = 'tstone'
move_arm = 'psm2'
INTER_NUM = 50 # interpolate points number
DLC_CONFIG_PATH = "/home/ben/ssd/code/robot/accel-challenge/accel_challenge/challenge2/data/dlc/dlc_calibrate-1-2022-04-20/config.yaml"
TEST_IMAGE_FILE_DIR = "/home/ben/ssd/code/robot/accel-challenge/accel_challenge/challenge2/data/dlc/dlc_calibrate-1-2022-04-20/labeled-data/calibrate_record20220420T000725/img3035.png"
ERROR_DATA_DIR = "/home/ben/ssd/code/robot/accel-challenge/accel_challenge/challenge2/data/error_data"
load_dict = {'dlc_config_path':DLC_CONFIG_PATH,
            'keras_model_path':str(Path(ERROR_DATA_DIR) / 'model.hdf5'),
            'scalers_path':str(Path(ERROR_DATA_DIR) / 'scalers.pkl')}



# #===self testing (comment when evaluation)
# # joint_calibrate_offset_gt = np.array(np.deg2rad([1,2,0,0,0,0]))
# # joint_calibrate_offset_gt[2] = 0.02
# joint_calibrate_offset_gt = np.array([0,0,0,0,0,0])
# print("setting error: ", joint_calibrate_offset_gt)
# set_error(move_arm, joint_calibrate_offset_gt.tolist())
# needle_pub = rospy.Publisher('/CRTK/Needle/servo_cp', PoseStamped)
# needle_zero_force_pub = rospy.Publisher('/CRTK/Needle/zero_force', Bool)
# rate = rospy.Rate(100)
# needle_pos0 = [-0.20786757338201337, 0.5619611862776279, 0.7517253877244148]
# # needle_rpy0 = [0.03031654271074325, 0.029994510295635185, -0.00018838556827461113]
# needle_rpy0 = [0.03031654271074325, 0.029994510295635185, np.deg2rad(-180)]
# time.sleep(1)
# for i in range(2):
#     msg = T2PoseStamped(RPY2T(*needle_pos0, *needle_rpy0))
#     needle_pub.publish(msg)
#     rate.sleep()
# time.sleep(1)
# for i in range(2):
#     msg = Bool()
#     msg.data = True
#     needle_zero_force_pub.publish(msg)
#     rate.sleep()
# time.sleep(1)


#=== initial objects
engine = ClientEngine()
engine.add_clients(['psm2','ecm', 'scene'])
engine.start()
# joint_calibrate_offset[0] = -0.2
print("reset pose..")
engine.clients[move_arm].reset_pose(walltime=None)
engine.clients[move_arm].wait()
engine.clients['ecm'].move_ecm_jp([0,0,0,0])
time.sleep(0.3)
task_pub = rospy.Publisher('/surgical_robotics_challenge/completion_report/'+team_name+'/task2', Bool)


#=== calulating grasp pose
# w = engine.clients['ambf'].client.get_world_handle()
# w.reset_bodies()
# engine.clients['ambf'].reset_all()
# time.sleep(1)
# engine.clients['ambf'].set_needle_pose(needle_pos0, needle_rpy0)
# measure meedle intial pose
T_n_w0 = engine.get_signal('scene', 'measured_needle_cp')
print("needle initial p", T_n_w0.p)
print("needle initial rpy", T_n_w0.M.GetRPY())
print("hover to needle..")
_, _, _Y = T_n_w0.M.GetRPY()
_offset_theta = pi/2
_Y +=_offset_theta
#print("Yaw angle :",  _Y*180/pi)
if _Y > pi /2: # project to -pi to pi range
    _Y = _Y -pi
elif _Y < -pi /2:
    _Y = _Y + pi
grasp_R = Rotation.RPY(*[0, 0, _Y]) * Rotation.RPY(*[pi, 0, 0]) 
T_g_w_dsr = Frame(grasp_R, T_n_w0.p)





#=========== move to hover pose
T_g_w_dsr = T_hover_gt * T_g_w_dsr # desire tool pose
T_HOVER_POSE = T_g_w_dsr
engine.clients[move_arm].servo_tool_cp(T_g_w_dsr, 100)
engine.clients[move_arm].wait()
time.sleep(0.2)
T_g_w_dsr_prv = T_g_w_dsr
T_g_w_dsr = None
#print("====")
#print("move to hover pose..")
#print(T_g_w_dsr_prv)
# time.sleep(1)


#============ approach needle
print("approach needle..")
T_target_w = T_n_w0 * T_gt_n
# T_target_w = engine.get_signal('scene', 'measured_needle_cp') * T_tip_n
# grasp_R =T_tip_n.M *  grasp_R
# T_target_w = engine.get_signal('scene', 'measured_needle_cp') * RPY2T(*[0,NEEDLE_R,0,0,0,0])
T_TARGET_POSE = T_target_w
#print("====")
#print("approach to grasp target..")
print("grasp..")
# T_g_w_dsr = Frame(grasp_R, T_TARGET_POSE.p)
T_g_w_dsr = T_n_w0 * T_tip_n
engine.clients[move_arm].servo_tool_cp(T_g_w_dsr, 200)
engine.clients[move_arm].wait()
#print(T_g_w_dsr)
T_g_w_dsr_prv = T_g_w_dsr
T_g_w_dsr = None
time.sleep(0.2)
_T_dsr = T_g_w_dsr_prv
_T_dsr2 = engine.clients['psm2'].T_g_w_dsr
print("T error 2", (_T_dsr.p-_T_dsr2.p).Norm())
T_NEEDLE_GRASP = T_g_w_dsr_prv.Inverse() * T_n_w0 # needle base pose w.r.t. gripper point

time.sleep(4)



#=== close engine 
engine.close()

