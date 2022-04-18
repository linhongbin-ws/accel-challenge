from accel_challenge.challenge2.ros_client import ClientEngine
from rospy import spin, Publisher, init_node, Rate
from time import sleep
from geometry_msgs.msg import PoseStamped
from accel_challenge.challenge2.tool import RPY2T, T2PoseStamped
from std_msgs.msg import Bool
# engine =  ClientEngine()
# # engine.add_clients(['ecm','psm1','psm2','scene','ambf'])
# engine.add_clients(['ambf'])
# engine.start()

# pos, rpy = engine.clients['scene'].get_needle_pose()
# print("is active:")
# print(f'pos:{pos}, rpy{rpy}')
sleep(1)
node = init_node('test',anonymous=True)
pub = Publisher('/CRTK/Needle/servo_cp', PoseStamped, queue_size=10)
pub2 = Publisher('/CRTK/Needle/zero_force', Bool, queue_size=10)
pos = [-0.20697696629671555, 0.5583796718976854, 0.8047253773270505]
rpy = [0.030316076843698688, 0.029994417468907717, 0.036854178797618514]
# engine.clients['ambf'].set_needle_pose(pos, rpy)
# engine.clients['ambf'].reset_needle()

msg = T2PoseStamped(RPY2T(*pos, *rpy))
rate = Rate(10) # 10hz
print(msg) 
for i in range(2):
    pub.publish(msg)
    rate.sleep()
sleep(1)
for i in range(2):
    msg = Bool()
    msg.data = True
    pub2.publish(msg)

print("ctrl c to stop")
spin()
# engine.close()