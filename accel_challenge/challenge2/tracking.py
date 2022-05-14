
import cv2
from deeplabcut.pose_estimation_tensorflow import predict
from matplotlib.pyplot import axis 
from skimage.util import img_as_ubyte
from deeplabcut.pose_estimation_tensorflow.config import load_config
from skimage import io
from deeplabcut.utils import auxiliaryfunctions
import numpy as np
from accel_challenge.challenge2.tool import PACKAGE_ROOT_PATH
from os.path import join
from os import listdir, sep
import numpy as np
from accel_challenge.challenge2.ros_client import ClientEngine
from time import sleep
import ros_numpy
from accel_challenge.challenge2.tool import PointCloud2_2_xyzNimage, render_rgb_xyz
import tensorflow as tf


class DLC_Predictor():
    def __init__(self, config_path) -> None:
        """ initialization is slow """
        # load config
        # config_path = join(PACKAGE_ROOT_PATH, "data","dlc_calibrate-dlc_calibrate-2022-01-24","config.yaml")
        physical_devices = tf.config.list_physical_devices('GPU')
        print(f"available device: {physical_devices}")
        tf.config.experimental.set_memory_growth(physical_devices[0], True)
        
        try:
            cfg = load_config(str(config_path))
            print(cfg)
        except FileNotFoundError:
            raise FileNotFoundError("It seems the model for shuffle s and trainFraction %s does not exist.")


        # load dlc config
        shuffle =1
        trainingsetindex = 0
        trainFraction = cfg['TrainingFraction'][trainingsetindex]
        modelfolder = join(cfg["project_path"], str(auxiliaryfunctions.get_model_folder(trainFraction, shuffle, cfg)))
        # print(modelfolder)
        path_test_config = join(modelfolder, 'test' ,'pose_cfg.yaml')
        try:
            dlc_cfg = load_config(str(path_test_config))
            print(dlc_cfg)
        except FileNotFoundError:
            raise FileNotFoundError(
                "It seems the model for shuffle %s and trainFraction %s does not exist." % (shuffle, trainFraction))


        # Check which snapshots are available and sort them by # iterations
        try:
            Snapshots = np.array(
                [fn.split('.')[0] for fn in listdir(join(modelfolder, 'train')) if "index" in fn])
        except FileNotFoundError:
            raise FileNotFoundError(
                "Snapshots not found! It seems the dataset for shuffle %s has not been trained/does not exist.\n Please train it before using it to analyze videos.\n Use the function 'train_network' to train the network for shuffle %s." % (
                    shuffle, shuffle))
        if cfg['snapshotindex'] == 'all':
            print(
                "Snapshotindex is set to 'all' in the config.yaml file. Running video analysis with all snapshots is very costly! Use the function 'evaluate_network' to choose the best the snapshot. For now, changing snapshot index to -1!")
            snapshotindex = -1
        else:
            snapshotindex = cfg['snapshotindex']
        increasing_indices = np.argsort([int(m.split('-')[1]) for m in Snapshots])
        Snapshots = Snapshots[increasing_indices]
        print("Using %s" % Snapshots[snapshotindex], "for model", modelfolder)


        dlc_cfg['init_weights'] = join(modelfolder, 'train', Snapshots[snapshotindex])
        print("weight path", dlc_cfg['init_weights'])
        trainingsiterations = (dlc_cfg['init_weights'].split(sep)[-1]).split('-')[-1]
        dlc_cfg['batch_size'] = cfg['batch_size']
        DLCscorer = auxiliaryfunctions.GetScorerName(cfg, shuffle, trainFraction, trainingsiterations=trainingsiterations)
        self.sess, self.inputs, self.outputs = predict.setup_GPUpose_prediction(dlc_cfg)
        self.pose_tensor = predict.extract_GPUprediction(self.outputs, dlc_cfg)
        print("Running ", DLCscorer, " with # of trainingiterations:", trainingsiterations)

        # print("sess",self.sess)



    def predict(self, input_image, input_depth_xyz=None):

        if isinstance(input_image, np.ndarray):
            for _ in range(8):
                _image = np.expand_dims(input_image, axis=0)
        elif isinstance(input_image, str):
                image = io.imread(input_image, plugin='matplotlib')
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                image = img_as_ubyte(image)
                _image = np.expand_dims(image, axis=0)
        else:
            raise NotImplementedError

        images = None
        for _ in range(8):
            # _image = np.expand_dims(image, axis=0)
            images = _image if images is None else np.concatenate((images, _image), axis=0)

        # print(images.shape)
        # print(image.shape)
        # print(self.sess)
        pose = self.sess.run(self.pose_tensor, feed_dict={self.inputs: images.astype(float)})
        # print(pose)
        pose_list = [(pose[i][0], pose[i][1], pose[i][2]) for i in range(int(pose.shape[0]/8))]

        if input_depth_xyz is None:
            return pose_list
            # return np.array(pose_list)
            return pose
        else:
            feature_xyz = [[input_depth_xyz[int(pose[i][0]),int(pose[i][1]),0],
                            input_depth_xyz[int(pose[i][0]),int(pose[i][1]),1], 
                            input_depth_xyz[int(pose[i][0]),int(pose[i][1]),2]] for i in range(len(pose_list))]
            return (np.array(pose_list), np.array(feature_xyz))
    
    def render(self, input_image_dir, annotes=None,circle_size=8):
        import matplotlib.pyplot as plt
        import numpy as np
        from matplotlib.patches import Circle
        import matplotlib.cbook as cbook


        img = plt.imread(input_image_dir)

        # # Make some example data
        # x = np.random.rand(5)*img.shape[1]
        # y = np.random.rand(5)*img.shape[0]

        # Create a figure. Equal aspect so circles look circular
        fig,ax = plt.subplots(1)
        ax.grid(False)
        ax.set_aspect('equal')

        # Show the image
        ax.imshow(img)

        # Now, loop through coord arrays, and create a circle at each x,y pair
        if not annotes is None:
            for yy,xx, prob in annotes:
                circ = Circle((xx,yy),circle_size)
                ax.add_patch(circ)

        # Show the image
        plt.show()
        





    # for i in range(pose.shape[0]):
    #     print("x:{:.2f}, y:{:.2f}, likelyhood: {:.2f}".format(pose[i][0], pose[i][1],pose[i][2]))


# class CameraModel(): # not use now since depth cloud could provide frame directly
#     def __init__(self) -> None:
#         from accel_challenge.challenge2.param import f, fov_angle, cam_width, cam_height, u0, v0
#         self.kuv = cam_height /2 / np.tan(fov_angle/2) # meaning ku and kv, pixel to distance
#         self.fuv = f*self.kuv
#         self.T_cam_project = np.array([[self.fuv, 0, u0, 0],[0,self.fuv,v0,0],[0,0,self.fuv,0]]) # perspecitve projection matrix
    
#     def project_P_cam(self, x_cam, y_cam, z_cam):
#         """ project point in camera frame"""
#         _p = np.array([x_cam, y_cam, z_cam, 1]).reshape(-1, 1)
#         _out = self.T_cam_project.dot(_p)
#         _out = _out.reshape(-1)
#         return (_out[0]/_out[2], _out[1]/_out[2])

#     def inverse_project_P_cam(self, x, y, d):
#         """ reverse to solve the point that being projected in camera frame
        
#         refer to 
#         [1]https://stackoverflow.com/questions/17832238/kinect-intrinsic-parameters-from-field-of-view/18199938#18199938

#         about z buffer
#         [2]https://zhuanlan.zhihu.com/p/393643084 
#         """
#         z_cam = d * self.fuv / np.sqrt(x**2 + y**2 + self.fuv**2)
#         x_cam = x * z_cam / self.fuv
#         y_cam = x * z_cam / self.fuv
#         return (x_cam, y_cam, z_cam)

if __name__ == '__main__':

    TEST_PREDICT = False
    TEST_PROJECTION = False
    TEST_DEPTH_IMAGE = False
    TEST_PREDICT_WITH_DEPTH = True
    if TEST_PREDICT:
        input = "/home/ben/code/robot/gym_suture/data/calibration/alpha-beta-0-0/{}.jpeg".format(8)
        pose_list = dlc_predict(input)
        print(pose_list)

    if TEST_PROJECTION:
        cam_model = CameraModel()
        print(cam_model.project_P_cam(*[1,2,3]))

    if TEST_DEPTH_IMAGE:
        camera_engine = ClientEngine()
        camera_engine.add_clients(['ecm'])
        camera_engine.start()
        sleep(0.5)
        data = camera_engine.get_signal('ecm', 'cameraL_image_depth')
        # data.data = None
        # print(data.data[0:32])

        _data = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        _data_rgb = ros_numpy.point_cloud2.split_rgb_field(_data)
        rgb = np.array([_data_rgb['r'], _data_rgb['g'], _data_rgb['b']]).T.reshape(1080,1920,3)
        rgb = np.flip(rgb, axis=0)
        while(True):
            cv2.imshow('preview',rgb)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

        camera_engine.close()
    
    if TEST_PREDICT_WITH_DEPTH:
        camera_engine = ClientEngine()
        camera_engine.add_clients(['ecm'])
        camera_engine.start()
        sleep(0.5)
        data = camera_engine.get_signal('ecm', 'cameraL_image_depth')
        tmp = PointCloud2_2_xyzNimage(data)
        xyz, rgb = tmp[0], tmp[1]
        render_rgb_xyz(rgb, xyz)
        # print(xyz)
        # result = dlc_predict(rgb, xyz)
        # print(result)