from distutils.command.config import config
from distutils.log import error
from accel_challenge.challenge2.ros_client import ClientEngine
from rospy import spin, Publisher, init_node, Rate, is_shutdown
from time import sleep
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from pathlib import Path
import datetime
import time
from numpy import pi
import numpy as np
import argparse
from tqdm import tqdm
import io
from accel_challenge.challenge2.tool import RPY2T, T2PoseStamped
from accel_challenge.challenge2.tracking import DLC_Predictor

parser = argparse.ArgumentParser()
parser.add_argument('-p', required=True, type=int) # program type, 1 for recording, 2 for trajectory
args, remaining = parser.parse_known_args()
assert args.p in [1, 2, 3, 4, 5,6,7,8]
is_no_engine = args.p in [7,8]

#======init variables
if not is_no_engine:
    engine =  ClientEngine()
    engine.add_clients(['ecm','psm2'])
    engine.start()
FPS = 10
TRAIN_TRAJ_SEED = 77
TEST_TRAJ_SEED = 66
TRAJ_POINTS_NUMS = 600
video_dir = './data'
DLC_CONFIG_PATH = "/home/ben/ssd/code/robot/accel-challenge/accel_challenge/challenge2/data/dlc/dlc_calibrate-1-2022-04-20/config.yaml"
TEST_IMAGE_FILE_DIR = "/home/ben/ssd/code/robot/accel-challenge/accel_challenge/challenge2/data/dlc/dlc_calibrate-1-2022-04-20/labeled-data/calibrate_record20220420T000725/img3035.png"
ERROR_DATA_DIR = "/home/ben/ssd/code/robot/accel-challenge/accel_challenge/challenge2/data/error_data"
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


def joint_error_test(seed, save_data_dir=None):
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
    # pub_error([0.1,0.1,0.1, 0,0,0])

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
    if not save_data_dir is None:
        save_data_dir = Path(save_data_dir)
        save_data_dir.mkdir(parents=True, exist_ok=True)
    rng_error = np.random.RandomState(seed) # use local seed
    error_mag_arr = np.deg2rad([5,5,0, 0,0,0])
    error_mag_arr[2] = 0.05 # simulation unit, for insertion joint
    num = 0
    while not is_shutdown():
        num += 1
        _error = rng_error.uniform(-error_mag_arr,error_mag_arr)
        pub_error(_error)
        print("                                                  ",end='\r')
        print(f'num:{num} error: {_error}, ctrl+c to stop', end='\r')
        sleep(0.5)
        engine.clients['psm2'].servo_tool_cp(pose_origin,100)
        engine.clients['psm2'].wait()
        sleep(0.5)
        if not save_data_dir is None:
            data_dict = {}
            data_dict['error'] = _error
            data_dict['image'] = engine.get_signal('ecm','cameraL_image')
            timestamp = datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
            filename = save_data_dir / f'error_{timestamp}.npz'
            with io.BytesIO() as f1:
                np.savez_compressed(f1, **data_dict)
                f1.seek(0)
                with filename.open('wb') as f2:
                    f2.write(f1.read())

def dlc_predict_test(config_path, test_image_dir):
    dlc_predictor = DLC_Predictor(config_path)
    annotes = dlc_predictor.predict(test_image_dir)
    print(annotes)
    dlc_predictor.render(test_image_dir,annotes=annotes)

def make_dataset(data_dir):
    """ load dataset
    """
    directory = Path(data_dir).expanduser()
    data_buffer = []
    for filename in tqdm(reversed(sorted(directory.glob('**/*.npz')))):
        try:
            with filename.open('rb') as f:
                data = np.load(f, allow_pickle=True)
                data = {k: data[k] for k in data.keys()}
                data_buffer.append(data)
        except Exception as e:
            print(f'Could not load episode: {e}')
            continue
    #=== return as numpy array
    xs = []
    ys = []
    for d in data_buffer:
        x = np.array(d['feature']).reshape(-1)
        xs.append(x)
        y = np.array(d['error']).reshape(-1)
        ys.append(y)
    xs = np.stack(xs, axis=0)
    ys = np.stack(ys, axis=0)

    return (xs, ys)

def make_features(load_dir, save_dir, dlc_config_path):
    """ make features and save 
    """
    load_dir = Path(load_dir).expanduser()
    dlc_predictor = DLC_Predictor(dlc_config_path)
    save_ft_names = ['error', 'feature']
    save_data_dir = Path(save_dir)
    save_data_dir.mkdir(parents=True, exist_ok=True)
    for filename in tqdm(reversed(sorted(load_dir.glob('**/*.npz')))):
        # try:
        with filename.open('rb') as f:
            data = np.load(f, allow_pickle=True)
            data = {k: data[k] for k in data.keys()}
            data['feature'] = dlc_predictor.predict(data['image'])
            # print(data)
            # print(data['error'])
            # print(data['feature'])
            data_dict = {k:data[k] for k in save_ft_names} 
            with io.BytesIO() as f1:
                np.savez_compressed(f1, **data_dict)
                f1.seek(0)
                save_name = str(filename.stem) + '-ft' + '.npz'
                save_file_dir = save_data_dir / save_name
                with save_file_dir.open('wb') as f2:
                    f2.write(f1.read())
        # except Exception as e:
        #     print(f'Could not load episode: {e}')
        #     continue

def train_mlp(train_data_dir, test_data_dir):
    """ train mlp to describe mapping from features to errors
    """
    train_data = make_dataset(train_data_dir)
    test_data = make_dataset(test_data_dir)
    print(train_data[0])
    print(train_data[0].shape)
    print(train_data[1].shape)
    

if args.p == 1:
    cam_render_test(video_dir=video_dir)
elif args.p == 2:
    joint_error_test(seed=TRAIN_TRAJ_SEED)
elif args.p == 3:
    dlc_predict_test(config_path=DLC_CONFIG_PATH, test_image_dir=TEST_IMAGE_FILE_DIR)
elif args.p == 4:
    TRAIN_ERROR_DATA_DIR = Path(ERROR_DATA_DIR) / 'train'
    joint_error_test(seed=TRAIN_TRAJ_SEED, save_data_dir=TRAIN_ERROR_DATA_DIR)
elif args.p == 5:
    TEST_ERROR_DATA_DIR = Path(ERROR_DATA_DIR) / 'test'
    joint_error_test(seed=TEST_TRAJ_SEED, save_data_dir=TEST_ERROR_DATA_DIR)
elif args.p == 6:
    data_dir = Path(ERROR_DATA_DIR) / 'test'
    train_data = make_dataset(data_dir=data_dir, dlc_config_path=DLC_CONFIG_PATH)
    print(train_data)
elif args.p == 7:
    load_dir = Path(ERROR_DATA_DIR) / 'test'
    save_dir = Path(ERROR_DATA_DIR) / 'test_ft'
    make_features(load_dir, save_dir, dlc_config_path=DLC_CONFIG_PATH)
    load_dir = Path(ERROR_DATA_DIR) / 'train'
    save_dir = Path(ERROR_DATA_DIR) / 'train_ft'
    make_features(load_dir, save_dir, dlc_config_path=DLC_CONFIG_PATH)
elif args.p == 8:
    train_data_dir = Path(ERROR_DATA_DIR) / 'train_ft'
    test_data_dir = Path(ERROR_DATA_DIR) / 'test_ft'
    train_mlp(train_data_dir, test_data_dir)
# print("ctrl c to stop")
# spin()
if not is_no_engine:
    engine.close()