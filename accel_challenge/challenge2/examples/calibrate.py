from distutils.log import error
from accel_challenge.challenge2.ros_client import ClientEngine
from rospy import spin, Publisher, init_node, Rate
from time import sleep
from geometry_msgs.msg import PoseStamped
from accel_challenge.challenge2.tool import RPY2T, T2PoseStamped
from std_msgs.msg import Bool
from pathlib import Path
import datetime
import time
from numpy import pi
import numpy as np
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-p', required=True, type=int) # program type, 1 for recording, 2 for trajectory
args, remaining = parser.parse_known_args()
assert args.p in [1, 2]

#======init variables
engine =  ClientEngine()
engine.add_clients(['ecm','psm2'])
engine.start()
video_dir = './data'
FPS = 10
SEED = 77
TRAJ_POINTS_NUMS = 600

def cam_render_test(video_dir=None):
    """ render to check image render quality
    """
    import cv2 
    print("tab `q` to exit..")
    engine.clients['psm2'].reset_pose()
    engine.clients['psm2'].wait()
    q_dsr = engine.clients['psm2'].q_dsr
    is_move =False
    #==video setup
    if not video_dir is None:
        frame = engine.get_signal('ecm','cameraL_image')
        height, width, channel = frame.shape
        fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
        video_dir = Path(video_dir)
        video_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
        file_name = 'calibrate_record' + timestamp + '.mp4'
        print(f'recording to {video_dir / file_name}')
        video = cv2.VideoWriter(str(video_dir / file_name), fourcc, float(FPS), (int(width), int(height)))

    while(True):
        frame = engine.get_signal('ecm','cameraL_image')
        cv2.imshow('preview',frame) # Display the resulting frame
        if not video_dir is None:
            video.write(frame)
        if not is_move:
            q_dsr[2] +=0.2
            engine.clients['psm2'].servo_jp(q_dsr, interpolate_num=600,clear_queue=False)
            q_dsr[1] +=0.2
            engine.clients['psm2'].servo_jp(q_dsr, interpolate_num=600,clear_queue=False)
            is_move=True
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(max(1/FPS -0.03, 0))
    cv2.destroyAllWindows()
    if not video_dir is None:
        video.release()


def joint_error_test(num_points):
    from sensor_msgs.msg import ChannelFloat32
    engine.clients['psm2'].reset_pose()
    engine.clients['psm2'].wait()
    arm_name = 'psm2'
    error_pub = Publisher('/ambf/env/' + arm_name + '/errors_model/set_errors', ChannelFloat32)
    rate = Rate(100)
    sleep(0.5) # wait a bit to initialize publisher 
    def pub_error(data): 
        """ pub error
        """
        msg = ChannelFloat32()
        msg.values = data
        for i in range(2):
            error_pub.publish(msg)
            # print(msg)
            rate.sleep()
    pub_error([0.1,0.1,0.1, 0,0,0])

    # print("ecm moving...")
    # engine.clients['ecm'].move_ecm_jp([0,0,-.9,0], time_out=40) # will block until the camera stop moving
    T_cam_w = engine.get_signal('ecm','camera_frame_state')
    # print("cam frame:",  np.rad2deg(T_cam_w.M.GetRPY()))
    Cam_Roll = T_cam_w.M.GetRPY()[0]

    #=== calibrate psm2 
    x_origin, y_origin, z_origin = -0.211084,    0.560047 - 0.3,    0.706611 + 0.2
    pose_origin = RPY2T(*[x_origin, y_origin, z_origin, pi, -pi/2,0]) * RPY2T(*[0,0,0,0,0,+Cam_Roll-pi/2])
    engine.clients['psm2'].servo_tool_cp(pose_origin,100)
    engine.clients['psm2'].wait()

    #===== error variantional data
    rng_error = np.random.RandomState(SEED) # use local seed
    error_mag_arr = np.deg2rad([5,5,0, 0,0,0])
    error_mag_arr[2] = 0.05 # simulation unit, for insertion joint
    for i in range(num_points):
        _error = rng_error.uniform(-error_mag_arr,error_mag_arr)
        pub_error(_error)
        print(f'error: {_error}')
        sleep(1)
        engine.clients['psm2'].servo_tool_cp(pose_origin,100)
        engine.clients['psm2'].wait()



if args.p == 1:
    cam_render_test(video_dir=video_dir)
elif args.p == 2:
    joint_error_test(num_points=TRAJ_POINTS_NUMS)
# print("ctrl c to stop")
# spin()
engine.close()