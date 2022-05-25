from accel_challenge.challenge2.ros_client import ClientEngine

from accel_challenge.challenge2.tool import RPY2T, Quaternion2T, SE3_2_T, T2PoseStamped
from numpy import pi
import time
import matplotlib.pyplot as plt
import numpy as np
arm_name = 'psm2'
engine = ClientEngine()
engine.add_clients(['psm2'])
engine.clients[arm_name].Kp_servo_jp = 0
engine.clients[arm_name].Ki_servo_jp = 0
engine.start()

engine.clients[arm_name].reset_pose()
engine.clients[arm_name].wait()
time.sleep(1)
x_origin, y_origin, z_origin = -0.20,    0.56 - 0.3,    0.72 # for psm2
R_origin, P_origin, Y_origin = 1, -0.5, 0.1
pose_origin = RPY2T(*[x_origin, y_origin, z_origin, R_origin, P_origin, Y_origin])

engine.clients[arm_name].servo_tool_cp(pose_origin,100)

Ts = []
qs_dsr = []
qs_msr = []
for i in range(150):
    Ts.append(engine.clients[arm_name].T_g_w_msr)
    qs_dsr.append(engine.clients[arm_name]._q_dsr)
    qs_msr.append(engine.clients[arm_name].get_signal('measured_js'))
    time.sleep(0.01)
xs = [T.p.x() for T in Ts]
ys = [T.p.y() for T in Ts]
zs = [T.p.z() for T in Ts]
Rs = [T.M.GetRPY()[0] for T in Ts]
Ps = [T.M.GetRPY()[1] for T in Ts]
Ys = [T.M.GetRPY()[2] for T in Ts]
ts = [i*0.01 for i in range(len(Ts))]

fig, axs = plt.subplots(12)

axs[0].plot(ts, xs)
axs[0].plot(ts, [x_origin for _ in xs])
axs[1].plot(ts, ys)
axs[1].plot(ts, [y_origin for _ in ys])
axs[2].plot(ts, zs)
axs[2].plot(ts, [z_origin for _ in zs])
axs[3].plot(ts, Rs)
axs[3].plot(ts, [R_origin for _ in zs])
axs[4].plot(ts, Ps)
axs[4].plot(ts, [P_origin for _ in zs])
axs[5].plot(ts, Ys)
axs[5].plot(ts, [Y_origin for _ in zs])

for k in range(6):
    axs[6 + k].plot(ts, [qs_dsr[i][k] for i in range(len(ts))])
    axs[6 + k].plot(ts, [qs_msr[i][k] for i in range(len(ts))])

print("dynamic tracking error q ", np.rad2deg(np.array(qs_dsr[101]) - np.array(qs_msr[101])))
print("steady error q ", np.rad2deg(np.array(qs_dsr[-1]) - np.array(qs_msr[-1])))

plt.show()
engine.close()