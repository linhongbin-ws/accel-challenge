# from distutils.command.build import build
# from distutils.command.config import config
# from distutils.log import error
from accel_challenge.challenge2.ros_client import ClientEngine
from rospy import Publisher, init_node, Rate, is_shutdown
from time import sleep
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Bool
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
from sklearn.preprocessing import StandardScaler 
import pickle





ERROR_MAG_ARR = np.deg2rad([5,5,0, 0,0,0])
ERROR_MAG_ARR[2] = 0.05 # simulation unit, for insertion joint
FPS = 10
TRAIN_TRAJ_SEED = 77
TEST_TRAJ_SEED = 66
ONLINE_TEST_TRAJ_SEED = 55
TRAJ_POINTS_NUMS = 600
NET_INTER_DIM_LIST = [400,300,200]
VIDEO_DIR = './data'
DLC_CONFIG_PATH = "/home/ben/ssd/code/robot/accel-challenge/accel_challenge/challenge2/data/dlc/dlc_calibrate-1-2022-04-20/config.yaml"
TEST_IMAGE_FILE_DIR = "/home/ben/ssd/code/robot/accel-challenge/accel_challenge/challenge2/data/dlc/dlc_calibrate-1-2022-04-20/labeled-data/calibrate_record20220420T000725/img3035.png"
ERROR_DATA_DIR = "/home/ben/ssd/code/robot/accel-challenge/accel_challenge/challenge2/data/error_data"

x_origin, y_origin, z_origin = -0.211084,    0.560047 - 0.3,    0.706611 + 0.2 # for psm2
YAW = -0.8726640502948968
pose_origin = RPY2T(*[0,0.15,0.1,0,0,0]) * RPY2T(*[0.2,0,0,0,0,0]) * RPY2T(*[x_origin, y_origin, z_origin, pi, -pi/2,0]) * RPY2T(*[0,0,0,0,0,YAW]) 


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

def set_error(arm_name, data):
    from sensor_msgs.msg import ChannelFloat32
    init_node('ros_client_engine',anonymous=True)
    error_pub = Publisher('/ambf/env/' + arm_name + '/errors_model/set_errors', ChannelFloat32, queue_size=1)
    sleep(0.5) # wait a bit to initialize publisher 
    msg = ChannelFloat32()
    msg.values = data
    error_pub.publish(msg)
    sleep(0.2)
def calibrate_joint_error(_engine, load_dict,  arm_name='psm2'):
    models = {}
    import os
    os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
    from tensorflow.keras.models import load_model
    from tensorflow import device
    _engine.clients[arm_name].servo_tool_cp(pose_origin,100)
    start = time.time()
    with device('/cpu'):
        models['dlc_predictor'] = DLC_Predictor(load_dict['dlc_config_path'])
        print("DLC_Predictor intialize time",time.time() - start)
        models['keras_model'] = load_model(load_dict['keras_model_path'])
        scalers =  pickle.load(open(load_dict['scalers_path'],'rb'))
    models['input_scaler'] = scalers['input_scaler']
    models['output_scaler'] = scalers['output_scaler']
    # print(models)
    # _engine.clients[arm_name].reset_pose()
    # _engine.clients[arm_name].wait()
    # T_cam_w = _engine.get_signal('ecm','camera_frame_state')
    # x_origin, y_origin, z_origin = -0.211084,    0.560047 - 0.3,    0.706611 + 0.2 # for psm2
    # # Cam_Roll = T_cam_w.M.GetRPY()[0]
    # # YAW = Cam_Roll-pi/2
    # YAW = -0.8726640502948968
    # pose_origin = RPY2T(*[0,0.15,0.1,0,0,0]) * RPY2T(*[0.2,0,0,0,0,0]) * RPY2T(*[x_origin, y_origin, z_origin, pi, -pi/2,0]) * RPY2T(*[0,0,0,0,0,YAW]) 
    # print("Yaw is ", YAW)

    q = _engine.clients[arm_name].get_signal('measured_js')
    # print("q error", np.array(q) - np.array(_engine.clients[arm_name]._q_dsr))
    # print(_engine.clients[arm_name].kin.is_out_qlim(q))
    # print("psm dsr",  _engine.clients[arm_name].T_g_w_dsr.p)
    # print("psm msr",  _engine.clients[arm_name].T_g_w_msr.p)
    # print("error", (_engine.clients[arm_name].T_g_w_dsr.p - _engine.clients[arm_name].T_g_w_msr.p).Norm())
    # print("engine error set", _engine.clients[arm_name].joint_calibrate_offset)
    data = {}
    data['image'] = _engine.get_signal('ecm','cameraL_image')
    # print("image shape:",data['image'].shape)
    start = time.time()
    data['feature'] =  models['dlc_predictor'].predict(data['image'])
    # print("DLC_Predictor predict time",time.time() - start)
    # print(data['feature'])
    data['feature'] = np.array(data['feature'])[:,:2].reshape(1,-1)
    data['feature'] = models['input_scaler'].transform(data['feature'])
    # print("input norm", data['feature'])
    data['err_pred'] = models['keras_model'].predict(data['feature'])
    input_test = np.array([[-0.57622886,  0.590467,   -0.54848045,  0.69391084, -0.5523233,   0.5873567,  -0.5669961,   0.64122444]])
    # print("test output", models['keras_model'].predict(input_test))
    # print("pred norm", data['err_pred'])
    data['err_pred'] = models['output_scaler'].inverse_transform(data['err_pred'])
    return data['err_pred'].reshape(-1)


def joint_error_test(seed, _engine, save_data_dir=None, load_dict=None,  arm_name='psm2', is_predict=False):
    if not load_dict is None: 
        models = {}
        from tensorflow.keras.models import load_model
        models['dlc_predictor'] = DLC_Predictor(load_dict['dlc_config_path'])
        models['keras_model'] = load_model(load_dict['keras_model_path'])
        scalers =  pickle.load(open(load_dict['scalers_path'],'rb'))
        models['input_scaler'] = scalers['input_scaler']
        models['output_scaler'] = scalers['output_scaler']
        print(models)

    from sensor_msgs.msg import ChannelFloat32
    _engine.clients[arm_name].reset_pose()
    _engine.clients[arm_name].wait()
    error_pub = Publisher('/ambf/env/' + arm_name + '/errors_model/set_errors', ChannelFloat32, queue_size=1)
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
    # _engine.clients['ecm'].move_ecm_jp([0,0,-.9,0], time_out=40) # will block until the camera stop moving
    # x_origin, y_origin, z_origin = -0.211084,    0.560047 - 0.3,    0.706611 + 0.2 # for psm2
    # # Cam_Roll = T_cam_w.M.GetRPY()[0]
    # # YAW = Cam_Roll-pi/2
    # YAW = -0.8726640502948968
    # pose_origin = RPY2T(*[0,0.15,0.1,0,0,0]) * RPY2T(*[0.2,0,0,0,0,0]) * RPY2T(*[x_origin, y_origin, z_origin, pi, -pi/2,0]) * RPY2T(*[0,0,0,0,0,YAW]) 
    _engine.clients[arm_name].servo_tool_cp(pose_origin,100)
    _engine.clients[arm_name].wait()

    #===== error variantional data
    if not save_data_dir is None:
        save_data_dir = Path(save_data_dir)
        save_data_dir.mkdir(parents=True, exist_ok=True)
    rng_error = np.random.RandomState(seed) # use local seed
    num = 0
    while not is_shutdown():
        num += 1
        if not is_predict:
            _error = rng_error.uniform(-ERROR_MAG_ARR,ERROR_MAG_ARR)
            if num<=1003:
                continue
            q_msr =  _engine.clients[arm_name].get_signal('measured_js')
            pub_error(_error)
            print(f'num:{num} error: {_error}, ctrl+c to stop')
            # q_msr =  _engine.clients[arm_name].get_signal('measured_js')
            # q_dsr = _error + np.array(q_msr)
            # _engine.clients[arm_name].servo_jp(q_dsr.tolist())
            sleep(0.1)
        _engine.clients[arm_name].servo_tool_cp(pose_origin,50)
        _engine.clients[arm_name].wait()
        sleep(3)
        # print("T error", (_engine.clients[arm_name].T_g_w_dsr.p - _engine.clients[arm_name].T_g_w_msr.p).Norm())

        if (not load_dict is None) and is_predict:
            data = {}
            data['image'] = _engine.get_signal('ecm','cameraL_image')
            data['feature'] =  models['dlc_predictor'].predict(data['image'])
            data['feature'] = np.array(data['feature'])[:,:2].reshape(1,-1)
            data['err_pred'] = models['keras_model'].predict(models['input_scaler'].transform(data['feature']))
            data['err_pred'] = models['output_scaler'].inverse_transform(data['err_pred'])
            return data['err_pred'].reshape(-1)

        if not load_dict is None:
            data = {}
            data['image'] = _engine.get_signal('ecm','cameraL_image')
            data['feature'] =  models['dlc_predictor'].predict(data['image'])
            data['feature'] = np.array([np.array(data['feature']).reshape(-1)])
            data['err_pred'] = models['keras_model'].predict(models['input_scaler'].transform(data['feature']))
            data['err_pred'] = models['output_scaler'].inverse_transform(data['err_pred'])
            print(data['err_pred'])
            print(_error[:3])

        if not save_data_dir is None:
            data_dict = {}
            data_dict['error'] = _error
            print("save data..")
            data_dict['image'] = _engine.get_signal('ecm','cameraL_image')
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
        # x = np.array(d['feature']).reshape(-1)
        x = np.array(d['feature'][:,:2]).reshape(-1)
        xs.append(x)
        y = np.array(d['error']).reshape(-1)
        y = y[:3] # only the first 3 joints
        ys.append(y)
    xs = np.stack(xs, axis=0)
    ys = np.stack(ys, axis=0)
    
    # print("feature", d['feature'].shape)
    # print(d['feature'])
    # print("image shape",d['image'].shape)
    # print(x)
    input_scaler = StandardScaler()
    input_scaler.fit(xs)
    xs_norm = input_scaler.transform(xs)
    output_scaler = StandardScaler()
    output_scaler.fit(ys)
    ys_norm = output_scaler.transform(ys)
        
    return (xs, ys, xs_norm, ys_norm, input_scaler, output_scaler)

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
            # print(data['image'].shape)
            # return
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

def train_mlp(train_data_dir, test_data_dir, save_model_dir):
    """ train mlp to describe mapping from features to errors
    """
    from tensorflow.keras.callbacks import ModelCheckpoint,EarlyStopping,ReduceLROnPlateau
    train_data = make_dataset(train_data_dir)
    pickle.dump({'input_scaler':train_data[4], 'output_scaler':train_data[5]} ,open(str(Path(save_model_dir) / 'scalers.pkl'),'wb'))

    test_data = make_dataset(test_data_dir)
    print(train_data[2])
    print(train_data[3])
    print(train_data[0].shape)
    print(train_data[1].shape)

    model = build_keras_model(input_dim=train_data[2].shape[1],
                              output_dim=train_data[3].shape[1], 
                              inter_dim_list=NET_INTER_DIM_LIST, 
                              lr=0.0001)

    
    callbacks = [ModelCheckpoint(filepath = str(Path(save_model_dir) / 'model.hdf5'), monitor='val_loss', verbose=0, mode='auto', save_best_only=False),
                EarlyStopping(monitor = 'val_loss', patience = 100),
                ReduceLROnPlateau(monitor = 'val_loss', factor = 0.5, patience = 5)]
    history = model.fit(train_data[2], train_data[3], validation_split = 0.2, epochs = 1000, verbose = 2, callbacks=callbacks)
    # print(model.predict(test_data[2]))
    test_pred = model.predict(test_data[2])
    test_pred_err = np.abs(test_data[5].inverse_transform(test_pred)-test_data[1])
    print(f"test error mean: {np.mean(test_pred_err)}, std:{np.std(test_pred_err)} ")
    print(f"test error mean (Deg): {np.rad2deg(np.mean(test_pred_err))}, std:{np.rad2deg(np.std(test_pred_err))} ")
    print("=================")
    print("test data input shape", test_data[0].shape)
    print("test data input [1]", test_data[0][0,:], test_data[2][0,:])
    print("test data output [1]", test_data[1][0,:], test_data[3][0,:])
    print("test data predict output [1]", test_data[5].inverse_transform(test_pred)[0,:], test_pred[0,:])
    scaler_load = pickle.load(open(str(Path(save_model_dir) / 'scalers.pkl'),'rb'))
    return
    # print(scaler_load['input_scaler'].mean_)

    # plt.plot(history.history['acc'])
    # plt.plot(history.history['val_acc'])
    # plt.legend(['training', 'validation'], loc = 'upper left')
    # plt.show()
    # results = model.evaluate(test_data[2], test_data[3])
    # print('Test RMS: ', np.sqrt(results))





def build_keras_model(input_dim, output_dim, inter_dim_list, lr, 
                        is_batchnormlize=False,
                        dropout_amount=None):
    from tensorflow.keras.models import Sequential
    from tensorflow.keras.layers import Activation, Dense, BatchNormalization, Dropout
    from tensorflow.keras import optimizers
    from tensorflow.keras.losses import MeanSquaredError
    model = Sequential()
    model.add(Dense(50, input_shape = (input_dim, ), kernel_initializer='he_normal'))
    if is_batchnormlize:
        model.add(BatchNormalization())
    model.add(Activation('relu'))
    if not dropout_amount is None:
        model.add(Dropout(dropout_amount))

    for inter_dim in inter_dim_list:
        model.add(Dense(inter_dim, kernel_initializer='he_normal'))
        if is_batchnormlize:
            model.add(BatchNormalization())
        model.add(Activation('relu'))    
        if not dropout_amount is None:
            model.add(Dropout(dropout_amount))
    model.add(Dense(output_dim, kernel_initializer='he_normal'))
    print(model.summary())
    adam = optimizers.Adam(lr = lr)
    model.compile(optimizer = adam, loss =MeanSquaredError())
    return model

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', required=True, type=int) # program type, 1 for recording, 2 for trajectory
    args, remaining = parser.parse_known_args()
    assert args.p in [1, 2, 3, 4, 5,6,7,8,9,10,11]
    is_no_engine = args.p in [7,8,9]

    #======init variables
    if not is_no_engine:
        engine =  ClientEngine()
        engine.add_clients(['ecm','psm2'])
        engine.start()

    if args.p == 1:
        cam_render_test(video_dir=VIDEO_DIR)
    elif args.p == 2:
        joint_error_test(seed=TRAIN_TRAJ_SEED, _engine=engine)
    elif args.p == 3:
        dlc_predict_test(config_path=DLC_CONFIG_PATH, test_image_dir=TEST_IMAGE_FILE_DIR)
    elif args.p == 4:
        engine.clients['psm2'].open_jaw()
        engine.clients['psm2'].wait()
        TRAIN_ERROR_DATA_DIR = Path(ERROR_DATA_DIR) / 'train'
        joint_error_test(seed=TRAIN_TRAJ_SEED, _engine=engine, save_data_dir=TRAIN_ERROR_DATA_DIR)
    elif args.p == 5:
        TEST_ERROR_DATA_DIR = Path(ERROR_DATA_DIR) / 'test'
        joint_error_test(seed=TEST_TRAJ_SEED, _engine=engine, save_data_dir=TEST_ERROR_DATA_DIR)
    elif args.p == 6:
        data_dir = Path(ERROR_DATA_DIR) / 'test_ft'
        train_data = make_dataset(data_dir=data_dir)
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
        train_mlp(train_data_dir, test_data_dir, ERROR_DATA_DIR)
    elif args.p == 9:
        import random
        image_dir_list = sorted((Path(ERROR_DATA_DIR) / 'train').glob('**/*.npz'))
        image_dir = random.choice(image_dir_list)
        print("render data: ",image_dir)
        with image_dir.open('rb') as f:
            data = np.load(f, allow_pickle=True)
            data = {k: data[k] for k in data.keys()}
        dlc_predict_test(config_path=DLC_CONFIG_PATH, test_image_dir=data['image'])

    elif args.p == 10:
        load_dict = {'dlc_config_path':DLC_CONFIG_PATH,
                    'keras_model_path':str(Path(ERROR_DATA_DIR) / 'model.hdf5'),
                    'scalers_path':str(Path(ERROR_DATA_DIR) / 'scalers.pkl')}
        error = joint_error_test(seed=ONLINE_TEST_TRAJ_SEED, _engine=engine, 
        load_dict=load_dict, is_predict=True)
        print("predict error deg :", np.rad2deg(error))
    elif args.p == 11:
        engine.clients['psm2'].open_jaw()
        engine.clients['psm2'].wait()
        error_gt = np.deg2rad([1, 2, 0, 0 ,0, 0])
        # error_gt = np.array([-0.00195975, -0.03527083, -0.013013,0,0,0])  
        set_error('psm2', error_gt.tolist())
        # set_error('psm2', [0,0, 0, 0,0,0])
        load_dict = {'dlc_config_path':DLC_CONFIG_PATH,
                    'keras_model_path':str(Path(ERROR_DATA_DIR) / 'model.hdf5'),
                    'scalers_path':str(Path(ERROR_DATA_DIR) / 'scalers.pkl')}
        error = calibrate_joint_error(_engine=engine, 
                              load_dict=load_dict,  arm_name='psm2')
        print("predict error (value)  (ground truth error):", error, error - error_gt[:3])
        print("predict error deg  (value)  (ground truth error):", np.rad2deg(error), np.rad2deg(error - error_gt[:3]))
        # dlc_predict_test(config_path=DLC_CONFIG_PATH, test_image_dir=TEST_IMAGE_FILE_DIR)
    # print("ctrl c to stop")
    # spin()
    if not is_no_engine:
        engine.close()