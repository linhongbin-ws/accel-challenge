import cv2
import numpy as np
from PyKDL import Frame, Rotation, Vector
from geometry_msgs.msg import TransformStamped, PoseStamped
from spatialmath.pose3d import SE3
from pathlib import Path
from os.path import dirname, normpath
import ros_numpy
from typing import Tuple

def get_package_root_abs_path():
    return normpath(dirname(Path(__file__).parent.absolute()))
PACKAGE_ROOT_PATH = get_package_root_abs_path()
cam_width, cam_height = 1920, 1080

#=====================opencv related
def resize_img(img, height):
    if isinstance(img, np.ndarray):
    # height = 300
    # method = cv2.INTER_NEAREST
    # method = cv2.INTER_LINEAR # fast
    # method = cv2.INTER_CUBIC # slow
        method = cv2.INTER_AREA # best way 
        img = cv2.resize(img, dsize=(int(height/0.5625), height), interpolation=method)
        return img
    else:
        raise NotImplementedError

def crop2square_img(img):
    size = img.shape
    if size[0]>size[1]:
        margin = int((size[0] - size[1])/2)
        return img[margin:margin+size[1],:,:]
    else:
        margin = int((size[1] - size[0])/2)
        return img[:,margin:margin+size[0],:]


#=====================PyKDL related
def gen_interpolate_frames(T_orgin, T_dsr, num):
    """ generate interpolate frames """
    T_delta =  T_orgin.Inverse()* T_dsr
    angle, axis = T_delta.M.GetRotAngle()
    return [T_orgin * Frame(Rotation.Rot(axis, angle*alpha), alpha*T_delta.p)  for alpha in np.linspace(0, 1,num=num).tolist()]

#===
def RPY2T(x,y, z, R, P, Y):
    return Frame(Rotation.RPY(*[R,P,Y]), Vector(*[x,y,z]))
def T2RPY(T: Frame)->Tuple[np.array]:
    pos = [T.p.x(),T.p.y(),T.p.z()]
    rpy = list(T.M.GetRPY())
    return np.array(pos), np.array(rpy)

#===
def Quaternion2T(x,y, z, rx, ry, rz, rw):
    return Frame(Rotation.Quaternion(*[rx, ry, rz, rw]),  Vector(*[x,y,z]))

#====
def PoseStamped2T(msg):
    """ Ros Message:PoseStamped to Frame"""
            
    x= msg.pose.position.x
    y= msg.pose.position.y
    z= msg.pose.position.z

    Qx = msg.pose.orientation.x
    Qy = msg.pose.orientation.y
    Qz = msg.pose.orientation.z
    Qw = msg.pose.orientation.w
    T = Quaternion2T(x, y, z, Qx,Qy,Qz,Qw)
    return T
def T2PoseStamped(T):
    """ 
    Frame to Ros Message:PoseStamped
    """
    msg = PoseStamped()
    rx,ry,rz,rw, = T.M.GetQuaternion()
    msg.pose.orientation.x = rx
    msg.pose.orientation.y = ry
    msg.pose.orientation.z = rz
    msg.pose.orientation.w = rw
    msg.pose.position.x = T.p.x()
    msg.pose.position.y = T.p.y()
    msg.pose.position.z = T.p.z()
    return msg 


#=======
def RigidBodyState2T(msg):
    """ Ros Message:RigidBodyState to Frame"""
            
    x= msg.pose.position.x
    y= msg.pose.position.y
    z= msg.pose.position.z

    Qx = msg.pose.orientation.x
    Qy = msg.pose.orientation.y
    Qz = msg.pose.orientation.z
    Qw = msg.pose.orientation.w
    T = Quaternion2T(x, y, z, Qx,Qy,Qz,Qw)
    return T
  
#====
def TransformStamped2T(msg):
    """ Ros Message:TransformStamped to Frame"""
            
    x= msg.transform.translation.x
    y= msg.transform.translation.y
    z= msg.transform.translation.z

    Qx = msg.transform.rotation.x
    Qy = msg.transform.rotation.y
    Qz = msg.transform.rotation.z
    Qw = msg.transform.rotation.w
    T = Quaternion2T(x, y, z, Qx,Qy,Qz,Qw)
    return T  
def T2TransformStamped(T):
    """ 
    Frame to Ros Message:TransformStamped
    """
    msg = TransformStamped()
    rx,ry,rz,rw, = T.M.GetQuaternion()
    msg.transform.rotation.x = rx
    msg.transform.rotation.y = ry
    msg.transform.rotation.z = rz
    msg.transform.rotation.w = rw
    msg.transform.translation.x = T.p.x()
    msg.transform.translation.y = T.p.y()
    msg.transform.translation.z = T.p.z()
    return msg 

#===
def SE3_2_T(SE3):
    R = SE3.R
    t = SE3.t
    return Frame(Rotation(Vector(*R[:,0].tolist()), Vector(*R[:,1].tolist()), Vector(*R[:,2].tolist())), Vector(*t.tolist()))
    # return Frame(Rotation(Vector(*[0,1,0]), Vector(*[2,1,0]), Vector(*[0,0,0])), Vector(*t.tolist()))
def T_2_SE3(T):
    R = T.M
    Rx = R.UnitX()
    Ry = R.UnitY()
    Rz = R.UnitZ()
    t = T.p
    _T = np.array([[Rx[0],Ry[0],Rz[0],t[0]],
                    [Rx[1],Ry[1],Rz[1],t[1]],
                    [Rx[2],Ry[2],Rz[2],t[2]],
                    [0,0,0,1]])
    return SE3(_T, check=False)

#====
def T_2_arr(T):
    R = T.M
    Rx = R.UnitX()
    Ry = R.UnitY()
    Rz = R.UnitZ()
    t = T.p
    _T = np.array([[Rx[0],Ry[0],Rz[0],t[0]],
                    [Rx[1],Ry[1],Rz[1],t[1]],
                    [Rx[2],Ry[2],Rz[2],t[2]],
                    [0,0,0,1]])
    return _T

#=====
def PointCloud2_2_xyzNimage(msg):
    _data = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    _data_rgb = ros_numpy.point_cloud2.split_rgb_field(_data)
    rgb = np.array([_data_rgb['r'], _data_rgb['g'], _data_rgb['b']]).T.reshape(cam_height, cam_width,3)
    rgb = np.flip(rgb, axis=0)

    xyz = np.array([_data_rgb['x'], _data_rgb['y'], _data_rgb['z']]).T.reshape(cam_height, cam_width,3)
    xyz = np.flip(xyz, axis=0)

    return (xyz, rgb)


# ======= render
def render_rgb(rgb_arr):
    print("press q to exit ..")
    while(True):
        cv2.imshow('preview',rgb_arr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    cv2.destroyAllWindows()

def render_rgb_xyz(rgb_arr, xyz_arr):
    import open3d as o3d 


    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz_arr.reshape(-1,3))
    # pcd.colors = o3d.utility.Vector3dVector(rgb_arr.reshape(-1,3))
    # o3d.visualization.draw_geometries([pcd])

    # color = o3d.geometry.Image(rgb_arr)
    # depth = o3d.geometry.Image(xyz_arr)
    # rgbd_image = o3d.geometry.RGBDImage.create_from_tum_format(
    #     rgb_arr, xyz_arr)
    # rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale=1.0, depth_trunc=50.0, convert_rgb_to_intensity=False)
    # pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    #     rgbd_image,
    #     o3d.camera.PinholeCameraIntrinsic(
    #         o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    o3d.visualization.draw_geometries([pcd])