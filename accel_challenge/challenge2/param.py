from PyKDL import Frame, Rotation, Vector
from accel_challenge.challenge2.tool import RPY2T
import numpy as np


# grasp_point_offset = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[-0.02, 0.005,-0.058])) # grasping point offset w.r.t. tool frame
# grasp_point_offset = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[0,0,0])) # grasping point offset w.r.t. tool frame
# grasp_point_offset = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[-0.02, 0,-0.058]))
# grasp_point_offset = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[-0.02, 0,0.058]))
# grasp_point_offset = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[-0.01, 0,0.05]))

# grasp_point_offset = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[0, 0, 0.09]))

# grasp_point_offset = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[0, 0, -0.005]))

grasp_point_offset = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[0, 0, -0.008]))

# grasp_point_offset = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[-0.02, 0.03,-0.058]))
# grasp_point_offset = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[0,0,-0]))

#==== grasp needle task

 # grasp target point offset w.r.t. needle frame
# T_gt_n = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[-0.09,0.03,0.]))
T_gt_n = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[-0.1,0.03,0.]))
# T_gt_n = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[-0.05,0.08,0.]))
T_hover_gt = RPY2T(0,0,0.3, 0, 0, 0)


#==== suture task
# NEEDLE_R = 0.13 # needle radius
NEEDLE_R = 0.103 # needle radius

T_tip_n = Frame(Rotation.RPY(*[0, 0, np.deg2rad(-30)]), Vector(*[0.055,0.083,0])) # tip frame w.r.t. needle base frame 0.04,0.07,0.
# T_tip_n = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[0.04,0.08/3,0])) # tip frame w.r.t. needle base frame



#============ tracking related

# instrinsic param
f = 0.01 # it is the distance to near plane, chech out http://www.songho.ca/opengl/gl_projectionmatrix.html
fov_angle = np.deg2rad(1.2) # check out /home/ben/code/robot/gym_suture/surgical_robotics_challenge/ADF/world/world_stereo.yaml
cam_width, cam_height = 1920, 1080
u0, v0 = cam_width/2, cam_height/2
kuv = cam_height /2 / np.tan(fov_angle/2) # meaning ku and kv, pixel to distance
fuv = f*kuv
T_cam_project = np.array([[fuv, 0, u0, 0],[0,fuv,v0,0],[0,0,fuv,0]]) # perspecitve projection matrix