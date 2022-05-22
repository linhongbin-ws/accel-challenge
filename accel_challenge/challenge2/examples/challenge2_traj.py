from accel_challenge.challenge2.ros_client import ClientEngine
from PyKDL import Frame, Rotation, Vector
from accel_challenge.challenge2.tool import RPY2T, Quaternion2T
from accel_challenge.challenge2.param import T_gt_n, T_hover_gt, NEEDLE_R, T_tip_n
from accel_challenge.challenge2.examples.calibrate import set_error
import time
import numpy as np
pi = np.pi





# params
move_arm = 'psm2'
set_error(move_arm, [0,0,0,0,0,0])
INTER_NUM = 50 # interpolate points number

# initial objects
engine = ClientEngine()
engine.add_clients(['psm1', 'psm2','ecm', 'scene', 'ambf'])
engine.start()

print("reset pose..")
engine.clients[move_arm].reset_pose(walltime=None)
engine.clients[move_arm].wait()
time.sleep(1)

# random needle position
needle_pos0 = [-0.20786757338201337, 0.5619611862776279, 0.7317253877244148]
needle_rpy0 = [0.03031654271074325, 0.029994510295635185, -0.00018838556827461113]
# w = engine.clients['ambf'].client.get_world_handle()
# w.reset_bodies()
time.sleep(1)
# engine.clients['ambf'].set_needle_pose(needle_pos0, needle_rpy0)

# measure meedle intial pose
T_n_w0 = engine.get_signal('scene', 'measured_needle_cp')

#============ logic of gripper rotational direction
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
time.sleep(1)
T_g_w_dsr_prv = T_g_w_dsr
T_g_w_dsr = None
#print("====")
#print("move to hover pose..")
#print(T_g_w_dsr_prv)
# time.sleep(1)


#============ approach needle
print("approach needle..")
T_target_w = engine.get_signal('scene', 'measured_needle_cp') * T_gt_n
# T_target_w = engine.get_signal('scene', 'measured_needle_cp') * T_tip_n
T_TARGET_POSE = T_target_w
#print("====")
#print("approach to grasp target..")
print("grasp..")
T_g_w_dsr = Frame(grasp_R, T_TARGET_POSE.p)
engine.clients[move_arm].servo_tool_cp(T_g_w_dsr, 300)
engine.clients[move_arm].wait()
#print(T_g_w_dsr)
T_g_w_dsr_prv = T_g_w_dsr
T_g_w_dsr = None
time.sleep(1)

#============ grasp
#print("====")
#print("grasp needle..")
engine.clients[move_arm].close_jaw()
engine.clients[move_arm].wait()
time.sleep(1)


#============ lift needle
time.sleep(0.5)
#print("====")
#print("lift needle..")
T_g_w_dsr = T_HOVER_POSE
print("hover..")
engine.clients[move_arm].servo_tool_cp(T_g_w_dsr, 300)
engine.clients[move_arm].wait()
#print(T_g_w_dsr)
time.sleep(1)
T_g_w_dsr_prv = T_g_w_dsr
T_NEEDLE_GRASP = T_g_w_dsr_prv.Inverse() * engine.get_signal('scene', 'measured_needle_cp') # needle base pose w.r.t. gripper point
T_g_w_dsr = None

print("needle tip pos: ", (engine.get_signal('scene', 'measured_needle_cp')*T_tip_n).p)
print("needle dsr pos: ", (T_g_w_dsr_prv*T_NEEDLE_GRASP*T_tip_n).p)

time.sleep(2)

#============= some calculation for suture
d =  np.abs((engine.get_signal('scene', 'measured_entry1_cp').p - engine.get_signal('scene', 'measured_exit1_cp').p).Norm())
r = NEEDLE_R# needle radius
theta = np.arcsin(d/2/r)
# pivot frame
Rx_pivot_w = (engine.get_signal('scene', 'measured_exit1_cp').p- engine.get_signal('scene', 'measured_entry1_cp').p)/d
Rz_pivot_w = Vector(*[0,0,1])
Ry_pivot_w = Rz_pivot_w * Rx_pivot_w # y_axis = z_axis cross product x_axis
p_pivot_w = (engine.get_signal('scene', 'measured_exit1_cp').p+ engine.get_signal('scene', 'measured_entry1_cp').p)/2 + Vector(0,0,r*np.cos(theta))
T_pivot_w = Frame(Rotation(Rx_pivot_w, Ry_pivot_w, Rz_pivot_w), p_pivot_w)
#print("===stats=======")
#print("exit frame is ",engine.get_signal('scene', 'measured_exit1_cp'))
#print("entry frame is ",engine.get_signal('scene', 'measured_entry1_cp'))
#print("r:", r,"  d:", d,"  theta: ",theta)
#print("pivot frame is ",T_pivot_w)
#print("needle frame is ",engine.get_signal('scene', 'measured_needle_cp'))
# needle entry and exit frame
TR_n_pivot = RPY2T(*[0,0,0,  -pi/2,0 ,0]) # Rotation Frame from pivot to needle base
# needle insertion interpolte trajectory frames
INSERTION_ITPL_NUM = 200
T_tip_w_ITPL_lst = [T_pivot_w * RPY2T(*[0,0,0, 0,theta-2*theta/INSERTION_ITPL_NUM*i,0]) * RPY2T(*[0,0,-r, 0,0,0]) * TR_n_pivot
                        for i in range(INSERTION_ITPL_NUM)] # interpolate frame from T_NET_w to T_NEX_w
T_NET_w = T_tip_w_ITPL_lst[0]# needle entry frame
T_NEX_w = T_tip_w_ITPL_lst[-1]# needle exit frame

print("measure entry", engine.get_signal('scene', 'measured_entry1_cp').p)
print("desire entry", T_NET_w.p)

print("measure entry", engine.get_signal('scene', 'measured_exit1_cp').p)
print("desire entry", T_NEX_w.p)

#================ move needle to entry point #1
#print("=============")
#print("move needle to approach entry#1")
print("move to entry #1..")
T_tip_w_dsr = T_NET_w
T_g_w_dsr = T_tip_w_dsr  * T_tip_n.Inverse() * T_NEEDLE_GRASP.Inverse()
engine.clients[move_arm].servo_tool_cp(T_g_w_dsr, 200)
engine.clients[move_arm].wait()
time.sleep(8)
print("entry frame pos: ", T_NET_w.p)
print("entry frame pos: ", engine.get_signal('scene', 'measured_entry1_cp').p)
print("needle tip dsr pos: ", (T_g_w_dsr*T_NEEDLE_GRASP*T_tip_n).p)
print("needle tip msr pos: ", (engine.get_signal('scene', 'measured_needle_cp')*T_tip_n).p)
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
    engine.clients[move_arm].wait()
time.sleep(2)
print("exit frame pos: ", T_NEX_w.p)
print("exit frame pos: ", engine.get_signal('scene', 'measured_exit1_cp').p)
print("needle tip dsr pos: ", (T_g_w_dsr*T_NEEDLE_GRASP*T_tip_n).p)
print("needle tip msr pos: ", (engine.get_signal('scene', 'measured_needle_cp')*T_tip_n).p)

#print("theta:", theta/pi*180)
engine.clients[move_arm].open_jaw()
engine.clients[move_arm].wait()



#=== close engine 
engine.close()

