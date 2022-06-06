from accel_challenge.challenge2.ros_client import ClientEngine
from PyKDL import Frame, Rotation, Vector
from accel_challenge.challenge2.tool import RPY2T, T2PoseStamped
from accel_challenge.challenge2.param import T_gt_n, T_hover_gt, NEEDLE_R, T_tip_n
from accel_challenge.challenge2.examples.calibrate import set_error, calibrate_joint_error
import time
import numpy as np
pi = np.pi

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool,Empty,Int32
import rospy
from surgical_robotics_challenge.task_completion_report import TaskCompletionReport
from pathlib import Path
#===params
team_name = 'tstone'
INTER_NUM = 50 # interpolate points number
DLC_CONFIG_PATH = "/home/ben/ssd/code/robot/accel-challenge/accel_challenge/challenge2/data/dlc/dlc_calibrate-1-2022-04-20/config.yaml"
TEST_IMAGE_FILE_DIR = "/home/ben/ssd/code/robot/accel-challenge/accel_challenge/challenge2/data/dlc/dlc_calibrate-1-2022-04-20/labeled-data/calibrate_record20220420T000725/img3035.png"
ERROR_DATA_DIR = "/home/ben/ssd/code/robot/accel-challenge/accel_challenge/challenge2/data/error_data"
load_dict = {'dlc_config_path':DLC_CONFIG_PATH,
            'keras_model_path':str(Path(ERROR_DATA_DIR) / 'model.hdf5'),
            'scalers_path':str(Path(ERROR_DATA_DIR) / 'scalers.pkl')}



INTER_NUM_SHORT = 300

engine = ClientEngine()

def task_3_init_finish_cb(msg):
    global is_finish_init
    is_finish_init = True


task_pub = rospy.Publisher('/surgical_robotics_challenge/completion_report/'+team_name+'/task3', Bool)
task3_init_pub = rospy.Publisher('/CRTK/scene/task_3_setup/init', Empty)
task3_init_finish_sub = rospy.Subscriber('/CRTK/scene/task_3_setup/ready',Empty, task_3_init_finish_cb, queue_size=1)
task3_estimation_num_pub = rospy.Publisher('/Tstone_msgs/estimation_num', Int32)
is_finish_init = False
time.sleep(0.3)
msg = Empty()
task3_init_pub.publish(msg)

while not is_finish_init:
    pass

del task3_init_finish_sub

#=== initial objects
engine.add_clients(['psm1', 'psm2', 'ecm', 'scene'])
engine.start()
# joint_calibrate_offset[0] = -0.2
print("reset pose..")
engine.clients['psm1'].grasp_point_offset = RPY2T(*[0,0, -0.030, 0,0,0])
engine.clients['psm2'].grasp_point_offset = RPY2T(*[0,0, -0.030, 0,0,0])
engine.clients['psm1'].reset_pose()
engine.clients['psm1'].wait()
engine.clients['ecm'].move_ecm_jp([0,0,0,0])
# print(engine.clients['psm1'].grasp_point_offset)

# #===self testing (comment when evaluation)
# # joint_calibrate_offset_gt = np.array(np.deg2rad([1,2,0,0,0,0]))
# # joint_calibrate_offset_gt[2] = 0.02
# joint_calibrate_offset_gt = np.array([0,0,0,0,0,0])
# print("setting error: ", joint_calibrate_offset_gt)
# set_error('psm1', joint_calibrate_offset_gt.tolist())
# set_error('psm2', joint_calibrate_offset_gt.tolist())
# time.sleep(2)


T_n_w0 = engine.get_signal('scene', 'measured_needle_cp')
T_NEEDLE_GRASP_PSM2 = engine.clients['psm2'].T_g_w_msr.Inverse() * T_n_w0 # needle base pose w.r.t. gripper point

engine.close_client('ecm')

x_origin, y_origin, z_origin = -0.211084, 0.560047 - 0.3, 0.706611 + 0.2  # for psm2
YAW = -0.8726640502948968
pose_origin_psm2 = RPY2T(*[0, 0.15, 0.1, 0, 0, 0]) * RPY2T(*[0.2, 0, 0, 0, 0, 0]) * RPY2T(*[x_origin, y_origin, z_origin, pi, -pi / 2, 0]) * RPY2T(*[0, 0, 0, 0, 0, YAW]) * RPY2T(*[0, 0, 0, -pi / 2, 0, 0])

needle_pos0 = [-0.15786, 0.0619, 0.7417]
needle_rpy0 = [0,0,0]
T_needle = RPY2T(*needle_pos0, *needle_rpy0)
T_g_w_init_dsr_PSM2 = T_needle * T_NEEDLE_GRASP_PSM2.Inverse()

for i in range(4):
    #============= some calculation for suture
    T_ENTRY = engine.get_signal('scene', "measured_entry{}_cp".format(i+1))
    T_EXIT = engine.get_signal('scene', "measured_exit{}_cp".format(i+1))
    # alpha = np.deg2rad(35)  # 5mm tip must be seen after penetrating exit hole
    alpha = np.deg2rad(30)  # 5mm tip must be seen after penetrating exit hole
    beta = np.deg2rad(140) # angle to extract needle
    d =  (T_ENTRY.p - T_EXIT.p).Norm()
    r = NEEDLE_R# needle radius
    theta = np.arcsin(d/2/r)
    # pivot frame
    Rx_pivot_w = T_EXIT.p - T_ENTRY.p
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
    theta_extract_list = np.linspace(-theta-alpha, -theta-beta , INSERTION_ITPL_NUM).tolist()
    T_tip_w_ITPL_extract_lst = [T_pivot_w * RPY2T(*[0,0,0, 0,theta,0]) * RPY2T(*[0,0,-r, 0,0,0]) * TR_n_pivot
                            for theta in theta_extract_list] # interpolate frame from T_NET_w to T_NEX_w                        
    T_NET_w = T_tip_w_ITPL_lst[0]# needle entry frame
    T_NEX_w = T_tip_w_ITPL_lst[-1]# needle exit frame
    print("entry dsr error 3:", T_NET_w.p-T_ENTRY.p)
    print("entry dsr error 3:", T_NEX_w.p-T_EXIT.p)



    #================ move needle to entry point #1
    #print("=============")
    #print("move needle to approach entry#1")
    print("move to entry #{}..".format(i+1))
    T_tip_w_dsr = T_NET_w
    T_g_w_dsr_PSM2 = T_tip_w_dsr  * T_tip_n.Inverse() * T_NEEDLE_GRASP_PSM2.Inverse()
    engine.clients['psm2'].servo_tool_cp(T_g_w_dsr_PSM2, INTER_NUM_SHORT)
    engine.clients['psm2'].wait()
    time.sleep(0.5)
    # _T_dsr = T_g_w_dsr_PSM2
    # _T_dsr2 = engine.clients['psm2'].T_g_w_dsr_PSM2
    # _T_msr = engine.clients['psm2'].T_g_w_msr
    # print("T error", (_T_msr.p-_T_dsr2.p).Norm())
    # print("T error 2", (_T_dsr.p-_T_dsr2.p).Norm())
    # time.sleep(0.5)
    # engine.clients['psm2'].servo_tool_cp(RPY2T(*[0,0,0.1,0,0,0]) * T_g_w_dsr_PSM2, 200)
    # engine.clients['psm2'].wait()


    #============ insert needle 
    print("insert..")
    T_tip_w_dsr_prv = T_tip_w_dsr
    for T_tip_w_dsr in T_tip_w_ITPL_lst:
        T_g_w_dsr_PSM2 = T_tip_w_dsr * T_tip_n.Inverse() * T_NEEDLE_GRASP_PSM2.Inverse() 
        # #print(T_g_w_dsr_PSM2)
        # #print(type(T_g_w_dsr_PSM2))
        engine.clients['psm2'].servo_tool_cp(T_g_w_dsr_PSM2, interpolate_num=None, clear_queue=False)
    engine.clients['psm2'].wait()
    time.sleep(2)
    # print("exit frame pos: ", T_NEX_w.p)
    # print("exit frame pos: ", engine.get_signal('scene', 'measured_exit1_cp').p)
    # _T_dsr = T_g_w_dsr_PSM2
    # _T_dsr2 = engine.clients['psm2'].T_g_w_dsr_PSM2
    # _T_msr = engine.clients['psm2'].T_g_w_msr
    # print("T error", (_T_msr.p-_T_dsr2.p).Norm())
    # print("T error 2", (_T_dsr.p-_T_dsr2.p).Norm())
    # print("theta:", theta/pi*180)




    #==== left arm extract needle
    T_gt_n_PSM1 = RPY2T(*[0.015,0.103,0, 0,0,0]) * RPY2T(*[0,0,0, -pi,0, np.deg2rad(10)]) # tip frame w.r.t. needle base frame 0.04,0.07,0.
    T_g_w_dsr_PSM1 = engine.clients['psm2'].T_g_w_msr * T_NEEDLE_GRASP_PSM2 * T_gt_n_PSM1 
    # print("needle dsr", T_tip_w_dsr.p)
    # print("needle msr", engine.get_signal('scene', 'measured_needle_cp').p)
    engine.clients['psm1'].servo_tool_cp(T_g_w_dsr_PSM1*RPY2T(0,0,-0.08,0,0,0), INTER_NUM_SHORT)
    engine.clients['psm1'].open_jaw()
    engine.clients['psm1'].wait()
    time.sleep(2)
    engine.clients['psm1'].servo_tool_cp(T_g_w_dsr_PSM1, INTER_NUM_SHORT)
    engine.clients['psm1'].wait()
    time.sleep(2)
    engine.clients['psm1'].close_jaw()
    engine.clients['psm1'].wait()
    time.sleep(3)
    T_NEEDLE_GRASP_PSM1 = engine.clients['psm1'].T_g_w_msr.Inverse() * engine.clients['psm2'].T_g_w_msr * T_NEEDLE_GRASP_PSM2
    engine.clients['psm2'].open_jaw()
    engine.clients['psm2'].wait()
    time.sleep(3)
    print("extract..")
    for T_tip_w_dsr in T_tip_w_ITPL_extract_lst:
        T_g_w_dsr_PSM1 = T_tip_w_dsr * T_tip_n.Inverse() * T_NEEDLE_GRASP_PSM1.Inverse() 
        # #print(T_g_w_dsr_PSM2)
        # #print(type(T_g_w_dsr_PSM2))
        engine.clients['psm1'].servo_tool_cp(T_g_w_dsr_PSM1, interpolate_num=None, clear_queue=False)

    engine.clients['psm1'].wait()
    time.sleep(1)

    #==lift
    print("lift")
    T_g_w_dsr_PSM1 = RPY2T(*[0,0,0.1,0,0,0]) *T_g_w_dsr_PSM1 
    engine.clients['psm1'].servo_tool_cp(T_g_w_dsr_PSM1, INTER_NUM_SHORT)
    engine.clients['psm1'].wait()
    T_g_w_dsr_PSM1 = T_g_w_dsr_PSM1 * RPY2T(*[0,0,0,0,0,-pi/4]) 
    engine.clients['psm1'].servo_tool_cp(T_g_w_dsr_PSM1, INTER_NUM_SHORT)
    engine.clients['psm1'].wait()
    time.sleep(1)

    #=== handover
    T_gt_n_PSM2 =  T_gt_n * RPY2T(*[0,0,0, -pi,0, -pi/2])
    T_g_w_dsr_PSM2 = engine.clients['psm1'].T_g_w_msr * T_NEEDLE_GRASP_PSM1 * T_gt_n_PSM2
    engine.clients['psm2'].servo_tool_cp(T_g_w_dsr_PSM2*RPY2T(0,0,-0.09,0,0,0), INTER_NUM_SHORT)
    engine.clients['psm2'].open_jaw()
    engine.clients['psm2'].wait()
    engine.clients['psm2'].servo_tool_cp(T_g_w_dsr_PSM2, INTER_NUM_SHORT)
    engine.clients['psm2'].wait()
    time.sleep(1)
    engine.clients['psm2'].close_jaw()
    engine.clients['psm2'].wait()
    time.sleep(0.5)
    T_NEEDLE_GRASP_PSM2 = engine.clients['psm2'].T_g_w_msr.Inverse() * engine.clients['psm1'].T_g_w_msr * T_NEEDLE_GRASP_PSM1
    engine.clients['psm1'].open_jaw()
    engine.clients['psm1'].wait()
    T_g_w_dsr_PSM2 = RPY2T(*[-0.1,0,0,0,0,0]) * T_g_w_dsr_PSM2 * RPY2T(*[0,0,0,0,0,-pi/4-pi/2]) 
    engine.clients['psm2'].servo_tool_cp(T_g_w_dsr_PSM2, INTER_NUM_SHORT)
    engine.clients['psm2'].wait()
    time.sleep(3)

    # put the needle in ground and wait for estimation
    engine.clients['psm2'].servo_tool_cp(T_g_w_init_dsr_PSM2, INTER_NUM_SHORT)
    engine.clients['psm2'].wait()
    engine.clients['psm2'].open_jaw()
    engine.clients['psm2'].wait()
    engine.clients['psm2'].reset_pose()
    engine.clients['psm2'].wait()
    time.sleep(3)
    task3_estimation_num_pub.publish(Int32(i+1))

time.sleep(2)
#=== send finish signal
for _ in range(2):
    msg = Bool()
    msg.data = True
    task_pub.publish(msg)
    time.sleep(0.01)
time.sleep(0.5)


# #===self testing, (comment when evaluation)
# engine.clients['psm2'].open_jaw()
# engine.clients['psm2'].wait()


engine.clients['psm1'].open_jaw()
engine.clients['psm2'].open_jaw()
engine.clients['psm1'].wait()
engine.clients['psm2'].wait()
#=== close engine 
engine.close()

