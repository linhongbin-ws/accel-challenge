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



#===self testing (comment when evaluation)
joint_calibrate_offset_gt = np.array(np.deg2rad([1,2,0,0,0,0]))
joint_calibrate_offset_gt[2] = 0.02
# joint_calibrate_offset = np.array([0,0,0,0,0,0])
print("setting error: ", joint_calibrate_offset_gt)
set_error(move_arm, joint_calibrate_offset_gt.tolist())
needle_pub = rospy.Publisher('/CRTK/Needle/servo_cp', PoseStamped)
needle_zero_force_pub = rospy.Publisher('/CRTK/Needle/zero_force', Bool)
rate = rospy.Rate(100)
needle_pos0 = [-0.20786757338201337, 0.5619611862776279, 0.7517253877244148]
# needle_rpy0 = [0.03031654271074325, 0.029994510295635185, -0.00018838556827461113]
needle_rpy0 = [0.03031654271074325, 0.029994510295635185, np.deg2rad(-180)]
time.sleep(1)
for i in range(2):
    msg = T2PoseStamped(RPY2T(*needle_pos0, *needle_rpy0))
    needle_pub.publish(msg)
    rate.sleep()
time.sleep(1)
for i in range(2):
    msg = Bool()
    msg.data = True
    needle_zero_force_pub.publish(msg)
    rate.sleep()
time.sleep(1)


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

#===== calibration
engine.clients[move_arm].joint_calibrate_offset = np.zeros(6)
error = calibrate_joint_error(_engine=engine, 
                        load_dict=load_dict,  arm_name=move_arm)
# print("predict error deg  (value):", np.rad2deg(error))
if 'joint_calibrate_offset_gt' in globals():
    print("predict error (value)  (ground truth error):", error, error - joint_calibrate_offset_gt[:3])
    print("predict error deg  (value)  (ground truth error):", np.rad2deg(error), np.rad2deg(error - joint_calibrate_offset_gt[:3]))
joint_calibrate_offset = np.concatenate((error, np.zeros(3)))
engine.clients[move_arm].joint_calibrate_offset = joint_calibrate_offset



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
T_g_w_dsr = Frame(grasp_R, T_TARGET_POSE.p)
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

#============ grasp
#print("====")
#print("grasp needle..")
engine.clients[move_arm].close_jaw()
engine.clients[move_arm].wait()
time.sleep(0.2)


#============ lift needle
#print("====")
#print("lift needle..")
T_g_w_dsr = T_HOVER_POSE
print("hover..")
engine.clients[move_arm].servo_tool_cp(T_g_w_dsr, 300)
engine.clients[move_arm].wait()
#print(T_g_w_dsr)
T_g_w_dsr_prv = T_g_w_dsr
T_g_w_dsr = None
time.sleep(0.5)
# msr = (engine.get_signal('scene', 'measured_needle_cp')*T_tip_n).p
# dsr = (T_g_w_dsr_prv*T_NEEDLE_GRASP*T_tip_n).p
# print("needle msr pos: ", msr)
# print("needle dsr pos: ", dsr)
# print("error:", (msr-dsr).Norm())
T_g_w_dsr = T_g_w_dsr_prv
_T_dsr = T_g_w_dsr_prv
_T_dsr2 = engine.clients['psm2'].T_g_w_dsr
print("T error 2", (_T_dsr.p-_T_dsr2.p).Norm())

#====== some pose testing (debugging)
# for i in range(2,6,1):
#     pose = [0,0,0, 0,0,0]
#     pose[i] = 0.4

#     pose[2]=0.1
#     T = T_g_w_dsr*RPY2T(*pose)
#     engine.clients[move_arm].servo_tool_cp(T, 100)
#     engine.clients[move_arm].wait()
#     time.sleep(0.3)
#     print("==")
#     msr = (engine.get_signal('scene', 'measured_needle_cp')*T_tip_n).p
#     dsr = (T*T_NEEDLE_GRASP*T_tip_n).p
    
#     print("needle msr pos: ", msr)
#     print("needle dsr pos: ", dsr)
#     print("error:", (msr-dsr).Norm())

#     _T_dsr = T
#     _T_dsr2 = engine.clients['psm2'].T_g_w_dsr
#     print("T error 2", (_T_dsr.p-_T_dsr2.p).Norm())



#============= some calculation for suture
T_ENTRY = engine.get_signal('scene', 'measured_entry1_cp') 
T_EXIT = engine.get_signal('scene', 'measured_exit1_cp')
alpha = np.deg2rad(35)  # 5mm tip must be seen after penetrating exit hole
d =  (T_ENTRY.p - T_EXIT.p).Norm()
r = NEEDLE_R# needle radius
theta = np.arcsin(d/2/r)
# pivot frame
Rx_pivot_w = T_EXIT.p- T_ENTRY.p
Rx_pivot_w = Rx_pivot_w / Rx_pivot_w.Norm()
Rz_pivot_w = Vector(*[0,0,1])
Ry_pivot_w = Rz_pivot_w * Rx_pivot_w # y_axis = z_axis cross product x_axis
p_pivot_w = (T_ENTRY.p+ T_EXIT.p)/2 + Vector(0,0,r*np.cos(theta))
T_pivot_w = Frame(Rotation(Rx_pivot_w, Ry_pivot_w, Rz_pivot_w), p_pivot_w)
#print("===stats=======")
#print("r:", r,"  d:", d,"  theta: ",theta)
#print("pivot frame is ",T_pivot_w)
#print("needle frame is ",engine.get_signal('scene', 'measured_needle_cp'))
# needle entry and exit frame
TR_n_pivot = RPY2T(*[0,0,0,  -pi/2,0 ,0]) # Rotation Frame from pivot to needle base
# needle insertion interpolte trajectory frames
INSERTION_ITPL_NUM = 400
theta_list = np.linspace(theta, -theta-alpha, INSERTION_ITPL_NUM).tolist()
T_tip_w_ITPL_lst = [T_pivot_w * RPY2T(*[0,0,0, 0,theta,0]) * RPY2T(*[0,0,-r, 0,0,0]) * TR_n_pivot
                        for theta in theta_list] # interpolate frame from T_NET_w to T_NEX_w
T_NET_w = T_tip_w_ITPL_lst[0]# needle entry frame
T_NEX_w = T_tip_w_ITPL_lst[-1]# needle exit frame
print("entry dsr error 3:", T_NET_w.p-T_ENTRY.p)
print("entry dsr error 3:", T_NEX_w.p-T_EXIT.p)


# #================ move needle to entry point #1 ready pose (for debugging)
# #print("=============")
# #print("move needle to approach entry#1")
# print("move to entry #1 ready pose ..")
# T_tip_w_dsr = RPY2T(*[-0.1, 0, 0, 0,0,0]) * T_NET_w 
# T_g_w_dsr = T_tip_w_dsr  * T_tip_n.Inverse() * T_NEEDLE_GRASP.Inverse()
# engine.clients[move_arm].servo_tool_cp(T_g_w_dsr, 200)
# engine.clients[move_arm].wait()
# time.sleep(2)
# msr = (engine.get_signal('scene', 'measured_needle_cp')*T_tip_n).p
# dsr = (T_g_w_dsr*T_NEEDLE_GRASP*T_tip_n).p
# dsr2 = (engine.clients['psm2'].T_g_w_msr*T_NEEDLE_GRASP*T_tip_n).p
# print("needle msr pos: ", msr)
# print("needle dsr pos: ", dsr)
# print("needle dsr2 pos: ", dsr2)
# print("needle error:", (msr-dsr).Norm())
# print("needle error 2:", (msr-dsr2).Norm())
# q = engine.clients['psm2'].get_signal('measured_js')
# print("q error",np.array(q)-np.array(engine.clients['psm2'].q_dsr))
# print("q", q)
# _T_dsr = T_g_w_dsr
# _T_dsr2 = engine.clients['psm2'].T_g_w_dsr
# _T_msr = engine.clients['psm2'].T_g_w_msr
# print("T error", (_T_msr.p-_T_dsr2.p).Norm())
# print("T error 2", (_T_dsr.p-_T_dsr2.p).Norm())
# print(engine.clients['psm2'].kin.is_out_qlim(q))

#================ move needle to entry point #1
#print("=============")
#print("move needle to approach entry#1")
print("move to entry #1..")
T_tip_w_dsr = T_NET_w
T_g_w_dsr = T_tip_w_dsr  * T_tip_n.Inverse() * T_NEEDLE_GRASP.Inverse()
engine.clients[move_arm].servo_tool_cp(T_g_w_dsr, 200)
engine.clients[move_arm].wait()
time.sleep(0.5)
# msr = (engine.get_signal('scene', 'measured_needle_cp')*T_tip_n).p
# dsr = (T_g_w_dsr*T_NEEDLE_GRASP*T_tip_n).p
# print("needle msr pos: ", msr)
# print("needle dsr pos: ", dsr)
# print("error:", (msr-dsr).Norm())
_T_dsr = T_g_w_dsr
_T_dsr2 = engine.clients['psm2'].T_g_w_dsr
_T_msr = engine.clients['psm2'].T_g_w_msr
print("T error", (_T_msr.p-_T_dsr2.p).Norm())
print("T error 2", (_T_dsr.p-_T_dsr2.p).Norm())
#print("T_tip_n ", T_tip_n)
#print("T_NEEDLE_GRASP ", T_NEEDLE_GRASP)

#============ insert needle 
print("insert..")
T_tip_w_dsr_prv = T_tip_w_dsr
for T_tip_w_dsr in T_tip_w_ITPL_lst:
    T_g_w_dsr = T_tip_w_dsr * T_tip_n.Inverse() * T_NEEDLE_GRASP.Inverse() 
    # #print(T_g_w_dsr)
    # #print(type(T_g_w_dsr))
    engine.clients[move_arm].servo_tool_cp(T_g_w_dsr, interpolate_num=None, clear_queue=False)
    # engine.clients[move_arm].wait()
    # msr = (engine.get_signal('scene', 'measured_needle_cp')*T_tip_n).p
    # dsr = (T_g_w_dsr*T_NEEDLE_GRASP*T_tip_n).p
    # print("needle error:", (msr-dsr).Norm())
engine.clients[move_arm].wait()
time.sleep(1)
print("exit frame pos: ", T_NEX_w.p)
print("exit frame pos: ", engine.get_signal('scene', 'measured_exit1_cp').p)
# msr = (engine.get_signal('scene', 'measured_needle_cp')*T_tip_n).p
# dsr = (T_NEX_w*T_NEEDLE_GRASP*T_tip_n).p
# print("needle msr pos: ", msr)
# print("needle dsr pos: ", dsr)
# print("error:", (msr-dsr).Norm())
_T_dsr = T_g_w_dsr
_T_dsr2 = engine.clients['psm2'].T_g_w_dsr
_T_msr = engine.clients['psm2'].T_g_w_msr
print("T error", (_T_msr.p-_T_dsr2.p).Norm())
print("T error 2", (_T_dsr.p-_T_dsr2.p).Norm())
print("theta:", theta/pi*180)

#=== send finish signal
for _ in range(2):
    msg = Bool()
    msg.data = True
    task_pub.publish(msg)
    rate.sleep()
time.sleep(3)


#===self testing, (comment when evaluation)
engine.clients[move_arm].open_jaw()
engine.clients[move_arm].wait()



#=== close engine 
engine.close()

