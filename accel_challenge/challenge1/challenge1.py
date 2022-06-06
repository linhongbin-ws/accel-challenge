from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty
import rospy
from surgical_robotics_challenge.camera import Camera
import tf_conversions.posemath as pm
# from surgical_robotics_challenge.evaluation import frame_to_pose_stamped_msg
from PyKDL import Frame, Rotation, Vector
from ambf_client import Client
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from argparse import ArgumentParser
import time
import numpy as np
import cv2
from skimage import morphology

# learning-based detection
import detectron2
from detectron2.utils.logger import setup_logger
setup_logger()

# import some common libraries
import numpy as np
import os, json, cv2, random, glob

# import some common detectron2 utilities
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog
from detectron2.data.datasets import register_coco_instances
from detectron2.engine import DefaultTrainer

import matplotlib.pyplot as plt
import torch
# from torch.autograd import Variable
import cv2
import math
import numpy as np
import time
torch.backends.cudnn.benchmark = True
import skimage
import matplotlib.pyplot as plt


class TaskCompletionReport:
    def __init__(self, team_name):
        self._team_name = team_name
        try:
            rospy.init_node('challenge_report_node')
        except:
            # Already initialized, so ignore
            done_nothing = True
        prefix = '/surgical_robotics_challenge/completion_report/' + self._team_name
        self._task1_pub = rospy.Publisher(prefix + '/task1/', PoseStamped, queue_size=1)
        self._task2_pub = rospy.Publisher(prefix + '/task2/', Bool, queue_size=1)
        self._task3_pub = rospy.Publisher(prefix + '/task3/', Bool, queue_size=1)

    def task_1_report(self, pose):
        print(self._team_name, 'reporting task 1 complete with result: ', pose)
        self._task1_pub.publish(pose)

    def task_2_report(self, complete):
        print(self._team_name, 'reporting task 2 complete with result: ', complete)
        self._task2_pub.publish(complete)

    def task_3_report(self, complete):
        print(self._team_name, 'reporting task 3 complete with result: ', complete)
        self._task3_pub.publish(complete)

class ImageLRSaver:
    def __init__(self):
        self.bridge = CvBridge()

        self.left_frame = None
        self.left_ts = None
        self.right_frame = None
        self.right_ts = None

        self.imgL_subs = rospy.Subscriber(
            "/ambf/env/cameras/cameraL/ImageData", Image, self.left_callback
        )
        self.imgR_subs = rospy.Subscriber(
            "/ambf/env/cameras/cameraR/ImageData", Image, self.right_callback
        )

        # Wait a until subscribers and publishers are ready
        rospy.sleep(1.5)

    def left_callback(self, msg):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.left_frame = cv2_img
            self.left_ts = msg.header.stamp
        except CvBridgeError as e:
            print(e)

    def right_callback(self, msg):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.right_frame = cv2_img
            self.right_ts = msg.header.stamp
        except CvBridgeError as e:
            print(e)

def get_max_IoU(pred_bboxes, gt_bbox):
    """
    given 1 gt bbox, >1 pred bboxes, return max iou score for the given gt bbox and pred_bboxes
    :param pred_bbox: predict bboxes coordinates, we need to find the max iou score with gt bbox for these pred bboxes
    :param gt_bbox: ground truth bbox coordinate
    :return: max iou score
    """

    # bbox should be valid, actually we should add more judgements, just ignore here...
    # assert ((abs(gt_bbox[2] - gt_bbox[0]) > 0) and
    #         (abs(gt_bbox[3] - gt_bbox[1]) > 0))

    if pred_bboxes.shape[0] > 0:
        # -----0---- get coordinates of inters, but with multiple predict bboxes
        ixmin = np.maximum(pred_bboxes[:, 0], gt_bbox[0])
        iymin = np.maximum(pred_bboxes[:, 1], gt_bbox[1])
        ixmax = np.minimum(pred_bboxes[:, 2], gt_bbox[2])
        iymax = np.minimum(pred_bboxes[:, 3], gt_bbox[3])
        iw = np.maximum(ixmax - ixmin + 1., 0.)
        ih = np.maximum(iymax - iymin + 1., 0.)

        # -----1----- intersection
        inters = iw * ih

        # -----2----- union, uni = S1 + S2 - inters
        uni = ((gt_bbox[2] - gt_bbox[0] + 1.) * (gt_bbox[3] - gt_bbox[1] + 1.) +
               (pred_bboxes[:, 2] - pred_bboxes[:, 0] + 1.) * (pred_bboxes[:, 3] - pred_bboxes[:, 1] + 1.) -
               inters)

        # -----3----- iou, get max score and max iou index
        overlaps = inters / uni
        ovmax = np.max(overlaps)
        jmax = np.argmax(overlaps)

    return overlaps, ovmax, jmax

def RT2Frame(R, t):
    """
    convert the R, t array into Frame type data
    """
    frame = Frame()
    R_list = R.reshape(-1,)
    t_list = t.reshape(-1,)
    frame.M = Rotation(R_list[0], R_list[1], R_list[2], R_list[3], R_list[4], R_list[5], R_list[6], R_list[7], R_list[8])
    frame.p = Vector(t_list[0], t_list[1], t_list[2])
    return frame

def frame_to_pose_stamped_msg(frame):
    """

    :param frame:
    :return:
    """
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = frame.p[0]
    msg.pose.position.y = frame.p[1]
    msg.pose.position.z = frame.p[2]

    msg.pose.orientation.x = frame.M.GetQuaternion()[0]
    msg.pose.orientation.y = frame.M.GetQuaternion()[1]
    msg.pose.orientation.z = frame.M.GetQuaternion()[2]
    msg.pose.orientation.w = frame.M.GetQuaternion()[3]

    return msg

def get_cus_Cross(pred_bboxes, gt_bbox):
    # -----0---- get coordinates of inters, but with multiple predict bboxes
    ixmin = np.maximum(pred_bboxes[:, 0], gt_bbox[0])
    iymin = np.maximum(pred_bboxes[:, 1], gt_bbox[1])
    ixmax = np.minimum(pred_bboxes[:, 2], gt_bbox[2])
    iymax = np.minimum(pred_bboxes[:, 3], gt_bbox[3])
    iw = np.maximum(ixmax - ixmin + 1., 0.)
    ih = np.maximum(iymax - iymin + 1., 0.)

    # -----1----- intersection
    inters = iw * ih

    # -----2 ---- own pred_bbox
    selfs = (pred_bboxes[:, 2] - pred_bboxes[:, 0] + 1.) * (pred_bboxes[:, 3] - pred_bboxes[:, 1] + 1.)

    # -----3----- self contain
    self_contain = inters / selfs

    # -----4----- get the satisfied index
    threshold = 0.8
    indexs = np.argwhere(inters / selfs > threshold)

    return self_contain, indexs

def dist_betw_pts(refer_pts, pts):
    refer_ = refer_pts.unsqueeze(dim=2)
    real = pts.unsqueeze(dim=0)
    real = real.unsqueeze(dim=0)
    batch_error = real - refer_

    channel_dist = torch.norm(batch_error[:, :, :], dim=3)
    #     min_dist = torch.min(channel_dist, dim=2).values
    min_dist = torch.min(channel_dist, dim=1).values
    sum_error = torch.sum(min_dist, dim=1)
    return sum_error

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-t', action='store', dest='team_name', help='Team Name', default='Tstone')
    parser.add_argument('-e', action='store', dest='task_report', help='Task to evaluate (1,2 or 3)', default=1)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    task_report = TaskCompletionReport(parsed_args.team_name)
    imgLR_saver = ImageLRSaver()

    client = Client('surgical_robotics_task_report')
    client.connect()
    time.sleep(0.3)

    task_to_report = int(parsed_args.task_report)
    if task_to_report not in [1, 2, 3]:
        raise Exception('ERROR! Acceptable task evaluation options (-e option) are 1, 2 or 3')

    # Get the stereo images
    while not rospy.is_shutdown():
        if imgLR_saver.left_frame is not None and imgLR_saver.right_frame is not None:
            imgL = imgLR_saver.left_frame
            imgR = imgLR_saver.right_frame
            break

    # Calculate opencv camera intrinsics
    fvg = 1.2
    width = 1920 #640
    height = 1080 #480
    f = height / (2 * np.tan(fvg / 2))

    intrinsic_params = np.zeros((3, 3))
    intrinsic_params[0, 0] = f
    intrinsic_params[1, 1] = f
    intrinsic_params[0, 2] = width / 2
    intrinsic_params[1, 2] = height / 2
    intrinsic_params[2, 2] = 1.0

    Kc = torch.tensor(intrinsic_params, dtype=torch.float32).cuda()

    print('Our method can achieve the summaration error under 1mm. \n' + 
     'If you cannot reproduce this results, try to use method 1, thux!')
    ############### Method1: pre-setup: stereo camera calibration parameters
    # print('loading the stereo calibration ...')
    # T_r_l = np.array([[9.96805112e-01,  7.41244200e-17, - 7.98722045e-02,  3.99680383e-02],
    #                   [-6.33718401e-17,  1.00000000e+00,  2.91066908e-17, -9.71445147e-17],
    #                   [7.98722045e-02, 2.58792641e-16, 9.96805112e-01, 1.59872153e-03],
    #                   [0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
    # T_l_f = np.array([[ 9.99200959e-01, -1.37194667e-17,  3.99680383e-02,  3.99680383e-02],
    #             [ 1.03959483e-17, -1.00000000e+00, -2.06547456e-16,  1.38777878e-16],
    #             [ 3.99680383e-02,  3.80487670e-16, -9.99200959e-01, -4.98801119e-01],
    #             [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
    ############### Method2: on-the-fly: stereo camera calibration parameters
    print('acquire the stereo calibration on-the-fly ...')
    ambf_cam_l = Camera(client, "cameraL")
    ambf_cam_r = Camera(client, "cameraR")
    ambf_cam_frame = Camera(client, "CameraFrame")
    T_FL = pm.toMatrix(ambf_cam_l.get_T_c_w())  # CamL to CamFrame
    T_FR = pm.toMatrix(ambf_cam_r.get_T_c_w())  # CamR to CamFrame
    # Convert AMBF camera axis to Opencv Camera axis
    F = np.array([[0, 1, 0, 0], [0, 0, -1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])
    T_LR = np.linalg.inv(T_FL).dot(T_FR)
    T_r_l = np.dot(F.dot(T_LR), np.linalg.inv(F))
    T_l_f = F.dot(np.linalg.inv(T_FL))
    ################################################################
    print('T_LR: \n', T_r_l)
    print('T_FL: \n', T_l_f)

    T_l_r = np.linalg.pinv(T_r_l)

    # load models
    print('loading models ...')
    cfg = get_cfg()
    cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
    # cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_101_FPN_3x.yaml"))
    cfg.DATASETS.TRAIN = ("needle_train",)
    cfg.DATASETS.TEST = ()  # no metrics implemented for this dataset
    cfg.DATALOADER.NUM_WORKERS = 8
    cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(
        "COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")  # Let training initialize from model zoo
    cfg.SOLVER.IMS_PER_BATCH = 4
    cfg.SOLVER.BASE_LR = 0.0015  # 0.015
    cfg.SOLVER.MAX_ITER = (
        30000
    )  # 300 iterations seems good enough, but you can certainly train longer
    cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = (
        128
    )  # faster, and good enough for this toy dataset
    cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1  # 3 classes (data, fig, hazelnut)
    # add the keypoint into prediction
    cfg.MODEL.KEYPOINT_ON = True
    cfg.MODEL.ROI_KEYPOINT_HEAD.NUM_KEYPOINTS = 2
    cfg.TEST.KEYPOINT_OKS_SIGMAS = np.ones((2, 1), dtype=float).tolist()
    cfg.MODEL.WEIGHTS = os.path.join(
        "./models/final.pth")  # path to the model we just trained
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5  # set a custom testing threshold
    predictor = DefaultPredictor(cfg)
    print('---model loaded!')

    # do prediction
    predictionsL = predictor(imgL)
    predictionsR = predictor(imgR)

    maskL = predictionsL['instances'].pred_masks[0]  # first 0 represent the index of instance
    keypointsL = predictionsL['instances'].pred_keypoints[0]
    scoresL = predictionsL['instances'].scores[0]
    [pts_yL, pts_xL] = torch.where(maskL)
    pixelsL = torch.hstack((pts_xL.reshape((-1, 1)), pts_yL.reshape((-1, 1))))

    keypointsR = predictionsR['instances'].pred_keypoints[0]
    scoresR = predictionsL['instances'].scores[0]

    # More accurate needle mask extraction
    condition1 = (imgL[:, :, 0] == imgL[:, :, 1])
    condition2 = (imgL[:, :, 1] == imgL[:, :, 2])
    condition3 = (imgL[:, :, 0] >= 10)  # suturing rgb value: 3,4,5
    needle_maskL = np.bitwise_and(condition1, condition2)
    maskL = np.bitwise_and(needle_maskL, condition3)

    [pts_yL, pts_xL] = torch.where(torch.tensor(maskL).cuda())
    pixelsL = torch.hstack((pts_xL.reshape((-1, 1)), pts_yL.reshape((-1, 1))))
    # More accurate needle mask extraction
    condition1 = (imgR[:, :, 0] == imgR[:, :, 1])
    condition2 = (imgR[:, :, 1] == imgR[:, :, 2])
    condition3 = (imgR[:, :, 0] >= 10)  # suturing rgb value: 3,4,5
    needle_maskR = np.bitwise_and(condition1, condition2)
    maskR = np.bitwise_and(needle_maskR, condition3)

    [pts_yR, pts_xR] = torch.where(torch.tensor(maskR).cuda())
    pixelsR = torch.hstack((pts_xR.reshape((-1, 1)), pts_yR.reshape((-1, 1))))

    # measure to acquire the needle position
    label_imgL = skimage.measure.label(maskL, connectivity=2)
    propsL = skimage.measure.regionprops(label_imgL)

    # measure to acquire the needle position
    label_imgR = skimage.measure.label(maskR, connectivity=2)
    propsR = skimage.measure.regionprops(label_imgR)
    gt_bboxR = predictionsR['instances'].pred_boxes[0].tensor.cpu().detach().numpy()  # top-left / bottom down
    gt_bboxR = np.squeeze(gt_bboxR)

    pred_bboxesR = np.empty((len(propsR), 4))
    for i in range(len(propsR)):
        pred_bboxesR[i, :] = np.array(
            [propsR[i]['bbox'][1], propsR[i]['bbox'][0], propsR[i]['bbox'][3], propsR[i]['bbox'][2]])
    overlapsR, ovmaxR, jmaxR = get_max_IoU(pred_bboxesR, gt_bboxR)
    selflapsR, indsR = get_cus_Cross(pred_bboxesR, gt_bboxR)
    full_mask_idR = np.unique(indsR, jmaxR)[0]

    maskR = np.zeros_like(label_imgR)
    if full_mask_idR != 0:
        for i in range(full_mask_idR.shape[0]):
            maskR = np.bitwise_or(maskR, label_imgR == (full_mask_idR[i] + 1))
    else:
        maskR = np.bitwise_or(maskR, label_imgR == (full_mask_idR + 1))

    [pts_yR, pts_xR] = torch.where(torch.tensor(maskR).cuda())
    pixelsR = torch.hstack((pts_xR.reshape((-1, 1)), pts_yR.reshape((-1, 1))))

    gt_bboxL = predictionsL['instances'].pred_boxes[0].tensor.cpu().detach().numpy()  # top-left / bottom down
    gt_bboxL = np.squeeze(gt_bboxL)

    pred_bboxesL = np.empty((len(propsL), 4))
    for i in range(len(propsL)):
        pred_bboxesL[i, :] = np.array(
            [propsL[i]['bbox'][1], propsL[i]['bbox'][0], propsL[i]['bbox'][3], propsL[i]['bbox'][2]])
    overlapsL, ovmaxL, jmaxL = get_max_IoU(pred_bboxesL, gt_bboxL)
    selflapsL, indsL = get_cus_Cross(pred_bboxesL, gt_bboxL)
    full_mask_idL = np.unique(indsL, jmaxL)[0]

    maskL = np.zeros_like(label_imgL)
    if full_mask_idL != 0:
        for i in range(full_mask_idL.shape[0]):
            maskL = np.bitwise_or(maskL, label_imgL == (full_mask_idL[i] + 1))
    else:
        maskL = np.bitwise_or(maskL, label_imgL == (full_mask_idL + 1))
    [pts_yL, pts_xL] = torch.where(torch.tensor(maskL).cuda())
    pixelsL = torch.hstack((pts_xL.reshape((-1, 1)), pts_yL.reshape((-1, 1))))

    # uniform sampling
    # time cost: 1000 pts--18s, 1500 pts:--39s, 3300 pts--42s, 6100 pts: 60s
    num_sample = 6000  # 500
    number_pixelsL = pixelsL.shape[0]
    if number_pixelsL > num_sample:
        pixels_selectL = np.random.uniform(0, number_pixelsL, num_sample)
        pixelsL = pixelsL[pixels_selectL, :]

    number_pixelsR = pixelsR.shape[0]
    if number_pixelsR > num_sample:
        pixels_selectR = np.random.uniform(0, number_pixelsR, num_sample)
        pixelsR = pixelsR[pixels_selectR, :]
    
    # print('pixelL/R size: {}/{}'.format(number_pixelsL, number_pixelsR))

    # spine model for needle
    l_st2end = 0.182886 # length from start point to end point
    l_st2end_tensor = torch.tensor([l_st2end]).cuda()

    ############################################## fine optimization
    # parral number to run the code
    parral_num_q1 = 8 if number_pixelsL < 3500 else 3 # 10
    parral_num_q2 = 8  # 10
    parral_num = parral_num_q1 * parral_num_q2
    tensor_1_col = torch.ones((parral_num, 1)).cuda()
    # better way for initialization
    q1_ws = np.linspace(np.pi / 6, np.pi / 3 * 2, parral_num_q1)
    q2_ws = np.linspace(-np.pi, np.pi, parral_num_q2)
    q1_init_, q2_init_ = np.meshgrid(q1_ws, q2_ws)
    q1_init = q1_init_.reshape((-1,))
    q2_init = q2_init_.reshape((-1,))

    q11 = torch.tensor(q1_init, dtype=torch.float32).cuda()
    q22 = torch.tensor(q2_init, dtype=torch.float32).cuda()

    if torch.max(torch.abs(keypointsL[0, :2] - keypointsL[1, :2])) > 10:
        ratio = 1.0
        s1_init = keypointsL[0, :2]
        s2_init = keypointsL[1, :2]
    elif torch.max(torch.abs(keypointsR[0, :2] - keypointsR[1, :2])) > 10:
        print('-----------------------using right frame keypoint instead ...')
        ratio = 6.0
        s1_init = keypointsR[0, :2]
        s2_init = keypointsR[1, :2]
    else:
        print('----------------------need to move the camera ...')
        print('here we calculate the coarse keypoint instead: faster speed ')
        ratio = 2.0
        skeleton0 = morphology.skeletonize(maskL)
        [pty, ptx] = np.where(skeleton0)
        sk_pt1 = torch.tensor([ptx[0], pty[0]]).cuda()
        sk_pt2 = torch.tensor([ptx[-1], pty[-1]]).cuda()

        # determine the order with the confidence
        if keypointsL[0, -1] > keypointsL[1, -1]:
            s1_init = keypointsL[0, :2]
            s2_init = sk_pt1 if torch.norm(s1_init - sk_pt1) > torch.norm(s1_init - sk_pt2) else sk_pt2
        else:
            s2_init = keypointsL[1, :2]
            s1_init = sk_pt1 if torch.norm(s2_init - sk_pt1) > torch.norm(s2_init - sk_pt2) else sk_pt2

        print('re-calculated keypoints position: ', s1_init, s2_init)

    s1 = torch.ones((parral_num, 1)).cuda() * s1_init
    s2 = torch.ones((parral_num, 1)).cuda() * s2_init

    q11.requires_grad = True
    q22.requires_grad = True
    s1.requires_grad = True
    s2.requires_grad = True

    # import torch_optimizer as optim

    param_dict = [{'params': q11, 'lr': 3e-3},  # degree
                  {'params': q22, 'lr': 3e-3},  # degree
                  {'params': s1, 'lr': 0.015*ratio},  # pixel
                  {'params': s2, 'lr': 0.015*ratio}]  # pixel
    optimizer = torch.optim.Adam(param_dict)

    R_l_r = torch.tensor(T_l_r[:3, :3], dtype=torch.float32).cuda()
    t_l_r = torch.tensor(T_l_r[:3, 3], dtype=torch.float32).cuda()

    t_s = time.time()
    best_batch_error = torch.tensor([1e6]).cuda()
    best_q1_hist = torch.tensor([math.pi]).cuda()
    best_q2_hist = torch.tensor([2 * math.pi]).cuda()
    best_cRn_hist = None
    best_ctn_hist = None
    # patience = 0
    # patience_threshold = 500
    tensor_1_col = torch.ones((parral_num, 1)).cuda()
    tensor_0 = torch.zeros(1).cuda()
    tensor_1 = torch.ones(1).cuda()
    a0 = torch.tensor([[1, 0, 0], [0, 0, 0], [0, 0, 0]], dtype=torch.float32).unsqueeze(dim=0).cuda()
    a1 = torch.tensor([[0, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=torch.float32).unsqueeze(dim=0).cuda()
    a2 = torch.tensor([[0, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=torch.float32).unsqueeze(dim=0).cuda()

    ax = torch.tensor([[1, 0, 0], [1, 0, 0], [1, 0, 0]], dtype=torch.float32).unsqueeze(dim=0).cuda()
    ay = torch.tensor([[0, 1, 0], [0, 1, 0], [0, 1, 0]], dtype=torch.float32).unsqueeze(dim=0).cuda()
    az = torch.tensor([[0, 0, 1], [0, 0, 1], [0, 0, 1]], dtype=torch.float32).unsqueeze(dim=0).cuda()

    # sample point in the reprojected ecllips
    Pt = np.loadtxt('./models/needle_axis_keypoints.txt').T # the origin of our coordinates is the center of the line (l_st2end)
    Pts = torch.tensor(Pt, dtype=torch.float32).cuda()
    print('---needle model loaded!')

    M = torch.tensor([[1, 0, 0], [0, 1, 0], [-intrinsic_params[0, 2], -intrinsic_params[1, 2], intrinsic_params[0, 0]]], dtype=torch.float32).cuda()

    for step in range(1500):  # 1500
        t_0 = time.time()
        s1_ = torch.cat((s1, tensor_1_col), dim=1)
        s2_ = torch.cat((s2, tensor_1_col), dim=1)
        c1 = torch.mm(s1_, M)  # start pt
        c2 = torch.mm(s2_, M)  # end pt
        n = torch.cross(c2, c1) / torch.norm(torch.cross(c2, c1), dim=1).unsqueeze(dim=1)
        alpha = torch.acos(torch.sum(torch.mul(c1, c2), dim=1) / torch.norm(c2, dim=1) / torch.norm(c1, dim=1))
        t_1 = time.time()
        h = l_st2end_tensor * torch.sin(q11)
        lambda2 = torch.div(h, torch.sin(alpha)).cuda()
        lambda1 = torch.div(h, torch.tan(alpha)) + torch.sign(q11 - math.pi / 2) * torch.sqrt(
            l_st2end_tensor * l_st2end_tensor - h * h)
        m1 = lambda1.unsqueeze(dim=1) * torch.div(c1, torch.norm(c1, dim=1).unsqueeze(dim=1))  # shape: [N, 3]
        m2 = lambda2.unsqueeze(dim=1) * torch.div(c2, torch.norm(c2, dim=1).unsqueeze(dim=1))  # shape: [N, 3]

        x1e = (m2 - m1) / torch.unsqueeze(torch.norm(m2 - m1, dim=1), dim=1)  # m2: start; m1: end (the needle tip)
        y_temp = torch.cross(n, x1e, dim=1)
        y1e = y_temp / torch.unsqueeze(torch.norm(y_temp, dim=1), dim=1)
        Rcx = torch.mul(x1e.unsqueeze(dim=1).permute(0, 2, 1), ax)
        Rcy = torch.mul(y1e.unsqueeze(dim=1).permute(0, 2, 1), ay)
        Rcz = torch.mul(n.unsqueeze(dim=1).permute(0, 2, 1), az)
        Rc1 = Rcx + Rcy + Rcz
        t_2 = time.time()
        # compute bR2 governed by q2
        cos = torch.cos(q22).unsqueeze(dim=1).unsqueeze(dim=2)
        sin = torch.sin(q22).unsqueeze(dim=1).unsqueeze(dim=2)
        R12 = a0 + torch.mul(a1, cos) + torch.mul(a2, sin)
        t_3 = time.time()
        # final pose of virtual needle w.r.t. camera frame
        cRn = torch.matmul(Rc1, R12)

        # Calculate ctn
        y1e_new = cRn[:, :, 1]
        ctn = 0.5 * (m1 + m2)

        # Create many arbitrary points to form a needle circle
        ###################################### Left projection
        temp = torch.matmul(cRn, Pts.unsqueeze(dim=0).repeat([parral_num, 1, 1])) + ctn.unsqueeze(dim=2)
        cprr = torch.div(temp, temp[:, 2, :].unsqueeze(dim=1))
        proj = torch.matmul(Kc, cprr[:, 0:3, :])  # x, y
        needle_proj_pts = torch.transpose(proj[:, 0:2, :], 2, 1)
        ###################################### Right projection
        cRn_r = torch.matmul(R_l_r, cRn)
        ctn_r = torch.matmul(R_l_r, ctn.unsqueeze(dim=2)) + t_l_r.unsqueeze(dim=1)
        temp_R = torch.matmul(cRn_r, Pts.unsqueeze(dim=0).repeat([parral_num, 1, 1])) + ctn_r
        cprr_R = torch.div(temp_R, temp_R[:, 2, :].unsqueeze(dim=1))
        proj_R = torch.matmul(Kc, cprr_R[:, 0:3, :])  # x, y
        needle_proj_pts_R = torch.transpose(proj_R[:, 0:2, :], 2, 1)

        batch_error = dist_betw_pts(needle_proj_pts, pixelsL) + dist_betw_pts(needle_proj_pts_R, pixelsR)
        # ignore the nan error
        batch_error = torch.where(torch.isnan(batch_error), torch.full_like(batch_error, 1e6), batch_error)

        loss = torch.sum(batch_error)

        optimizer.zero_grad()
        batch_error.backward(torch.ones_like(batch_error))
        t_4 = time.time()

        last_batch_error = batch_error
        last_q1_hist = q11.clone().detach()
        last_q2_hist = q22.clone().detach()

        optimizer.step()
        with torch.no_grad():
            q11[:] = torch.clamp(q11, 0, math.pi)
            q22[:] = torch.clamp(q22, -math.pi, math.pi)

        t_5 = time.time()

        select = torch.argmin(batch_error)

        if batch_error[select] < best_batch_error:
            best_batch_error = batch_error[select]
            best_q1_hist = q11[select].clone().detach()
            best_q2_hist = q22[select].clone().detach()
            best_cRn_hist = cRn[select].clone().detach()
            best_ctn_hist = ctn[select].clone().detach()

        if np.isnan(q11[select].item()) and np.isnan(q22[select].item()):
            # avoid the nan and return nan for q
            print('select, and batch_error: ', select, batch_error, batch_error[select])
            print('q11 select, q22 select : ', q11[select].item(), q22[select].item())
            print('***invalid estimation, exist with none: ', last_q1_hist)
            q11 = last_q1_hist
            q22 = last_q2_hist
            break

        if (step % 20 == 0):
            print(
                'iter: {}. min error: {}. q1: {} / q2: {}. selected index: {}'.format(step, batch_error[select].item(),
                                                                                      q11[
                                                                                          select].item() * 180 / math.pi,
                                                                                      q22[
                                                                                          select].item() * 180 / math.pi,
                                                                                      select.item()))

    t_e = time.time()
    print('totally cost {}s.'.format(t_e - t_s))
    print('*********************************************************************************************************************')
    print('*****************************************************Final Results***************************************************')
    select = torch.argmin(batch_error)
    print('min error: {}. q1: {} / q2: {}'.format(batch_error[select].item(), q11[select].item() * 180 / math.pi,
                                                  q22[select].item() * 180 / math.pi))
    print('s1 shape: ', s1.shape)
    print('keypoint1: x: {} y: {} / keypoint2: x: {} y: {}'.format(s1[select, 0].item(), s1[select, 1].item(), s2[select, 0].item(), s2[select, 1].item()))
    print('selected index: ', select.item())
    print('cRn: ', cRn[select, :, :])
    print('ctn: ', ctn[select, :])
    print('*********************************************************************************************************************')
    print('*********************************************************************************************************************')
    torch.cuda.empty_cache()

    pose_frame = RT2Frame(cRn[select, :, :].detach().cpu().numpy(), ctn[select, :].detach().cpu().numpy()) # needle in left frame

    # converted into world coordinates
    T_l_f_frame = RT2Frame(T_l_f[:3,:3], T_l_f[:3, 3])
    # transformation between self-defined needle coordination to ambf needle coordinates
    T_offset = np.array([[ 0.871982,    0.489403, -0.00136704, 0.00102883],
                        [ -0.489405,     0.87198, -0.00184491, -0.04541],
                        [ 0.000289144,  0.00227791,    0.999932, 0.00043177],
                        [ 0, 0, 0, 1]])
    T_offset_frame = RT2Frame(T_offset[:3,:3], T_offset[:3, 3])
    task_report.task_1_report(frame_to_pose_stamped_msg(T_l_f_frame.Inverse()*pose_frame*T_offset_frame))
    # task_report.task_1_report(frame_to_pose_stamped_msg(T_l_f_frame.Inverse()*pose_frame))
    print('final estimation: ', T_l_f_frame.Inverse()*pose_frame)
