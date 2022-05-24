## ros related
from sensor_msgs.msg import JointState, Image, PointCloud2, CompressedImage
from geometry_msgs.msg import TransformStamped, Transform, TwistStamped, PoseStamped
from rospy import Publisher, Subscriber, Rate, init_node, spin, get_published_topics, Rate, is_shutdown
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool

## common
from typing import List, Any
from time import sleep
from PyKDL import Frame, Rotation, Vector
from dataclasses import dataclass
import numpy as np
from time import sleep
from threading import Thread
import time
from queue import Queue

## ambf related
from ambf_msgs.msg import RigidBodyState, CameraState
### gym_suture related
from accel_challenge.challenge2.tool import Quaternion2T, RPY2T, gen_interpolate_frames, SE3_2_T, T_2_SE3, RigidBodyState2T, PoseStamped2T
from accel_challenge.challenge2.param import grasp_point_offset
from accel_challenge.challenge2.kinematics import PSM_KIN
from surgical_robotics_challenge.kinematics.psmIK import compute_IK

from simple_pid import PID


class ClientEngine():
    """
    an engine to manage clients that subscribe and publish CRTK related topics.
    """
    ROS_RATE = 100
    def __init__(self):
        self.clients = {}
        self.ros_node = init_node('ros_client_engine',anonymous=True)
        self.ros_rate = Rate(self.ROS_RATE)


    def add_clients(self, client_names:List[str]):
        for client_name in client_names:
            self._add_client(client_name)
        sleep(1) # wait for ros subscribed topics to be ok

    def start(self):
        for client in self.clients.values():
            client.start()
    
    def close(self):
        for client in self.clients.values():
            try:
                client.close()
            except Exception as e:
                print(str(e))

    def get_signal(self, client_name, signal_name):
        """
        get signal data from client
        """
        self._is_has_client(client_name, raise_error=True)
        return self.clients[client_name].get_signal(signal_name)

    def _add_client(self, client_name):
        if client_name in ['psm1', 'psm2']:
            self.clients[client_name] = PSMClient(self.ros_node, client_name)
        elif client_name == 'ecm':
            self.clients[client_name] = ECMClient(self.ros_node)
        elif client_name == 'scene':
            self.clients[client_name] = SceneClient(self.ros_node)
        elif client_name == 'ambf':
            self.clients[client_name] = AmbfClient()
        else:
            raise NotImplementedError

    def _is_has_client(self,client_name, raise_error=False):
        result = client_name in list(self.clients.keys())
        if raise_error and not result:
            raise Exception(f"client {client_name} has not been added")
        
    @property
    def client_names(self):
        return list(self.clients.keys())
    
@dataclass
class SignalData:
    """ data class for testing if signal is died"""
    data:Any
    counter:int


class BaseClient():
    """ 
    define some common property for clients 
    """
    COUNTER_BUFFER = 500
    def __init__(self, ros_node, is_run_thread=False):
        self.ros_node =ros_node
        self.sub_signals = {}
        self.subs = dict()
        self.pubs = dict()
        self.is_alive = False

        self.is_run_thread = is_run_thread
        if self.is_run_thread:
            self.thread = Thread(target=self.run) 
        else:
            self.thread = None

    def start(self):
        self.is_alive = True
        if self.is_run_thread:
            self.thread.start()
    def close(self):
        self.is_alive = False

    def _T2TransformStamped(self, T):
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

    def _TransformStamped2T(self, msg):
        """ 
        Ros Message:TransformStamped to Frame
        """
                
        x= msg.transform.translation.x
        y= msg.transform.translation.y
        z= msg.transform.translation.z

        Qx = msg.transform.rotation.x
        Qy = msg.transform.rotation.y
        Qz = msg.transform.rotation.z
        Qw = msg.transform.rotation.w
        T = Quaternion2T(x, y, z, Qx,Qy,Qz,Qw)
        return T  
    
    def _RigidBodyState2T(self, msg):
        """
        RigidBodyState message to Frame 
        """
        
        x= msg.pose.position.x
        y= msg.pose.position.y
        z= msg.pose.position.z

        Qx = msg.pose.orientation.x
        Qy = msg.pose.orientation.y
        Qz = msg.pose.orientation.z
        Qw = msg.pose.orientation.w
        T = Quaternion2T(x, y, z, Qx,Qy,Qz,Qw)

        return T

    def topic_wrap(self, topic_name, raise_exception=True):
        """ wrapper for ros topic name, check if the topic name exists
        
        raise exception if not exists
        """
        topics = [topic[0] for topic in get_published_topics()]
        result = topic_name in topics
        if (not result) and raise_exception:
            print(topics)
            raise Exception("topic {} does not exist, please check if crtk interface is running".format(topic_name))
        return topic_name
    
    
    def is_has_signal(self, signal_name:str):
        """
        check if signal name exists
        """
        return signal_name in self.sub_signals_names

    def set_signal(self, signal_name:str, data):
        """ 
        update signal data 
        """
        if not self.is_has_signal(signal_name):
            self.sub_signals[signal_name] = SignalData(data= data, counter=1)
        else:
            cnt = self.sub_signals[signal_name].counter
            cnt = cnt +1 if cnt<=self.COUNTER_BUFFER else self.COUNTER_BUFFER
            self.sub_signals[signal_name] = SignalData(data = data, counter = cnt)
    
    def get_signal(self, signal_name:str):
        """
        get signal data
        """
        if not self.is_has_signal(signal_name):
            print("signal_name is not exist")
            return None
        else:
            return self.sub_signals[signal_name].data

    def reset_signal_counter(self, signal_name:str):
        """
        set signal coutner to 0
        """
        if self.is_has_signal(signal_name):
            self.sub_signals[signal_name].counter = 0

    def run():
        raise NotImplementedError
    
    @property
    def sub_signals_names(self):
        return list(self.sub_signals.keys())
    @property
    def sub_topics_names(self):
        return list(self.subs.keys())
    @property
    def pub_topics_names(self):
        return list(self.pubs.keys())

class AmbfClient(BaseClient):
    def __init__(self, ros_node=None):
        super(AmbfClient, self).__init__(ros_node, is_run_thread=False)
        from ambf_client import Client
        self.client = Client('gym_suture')
        self.client.connect()

    def close(self):
        self.client.clean_up()
        self.is_alive = False

    def reset_all(self):
        """ reset all, similar to ctrl+R 
        """
        w = self.client.get_world_handle()
        time.sleep(0.1)
        w.reset() # Resets the whole world (Lights, Cams, Volumes, Rigid Bodies, Plugins etc)
        time.sleep(0.5)
        w.reset_bodies() # Reset Static / Dynamic Rigid Bodies

    def reset_needle(self,repeat=3, accuracy=0.09):
        """ reset needle to starting position 
        """
        pos = [-0.20786757338201337, 0.5619611862776279, 0.7317253877244148] # hardcode this position
        rpy = [0.03031654271074325, 0.029994510295635185, -0.00018838556827461113]
        # pos_ready = pos.copy()
        # pos_ready[1] = pos_ready[1] - 0.3
        # pos_ready[2] = pos_ready[2] + 0.2
        for i in range(repeat):
            if i+1 == repeat:
                self.reset_all()
            else:
                self.set_needle_pose(pos=pos, rpy=rpy)
            _pos, _rpy = self.get_needle_pose()
            err = np.linalg.norm(np.array(_pos)-np.array(pos))
            if err<accuracy: # ensure needle stay within target
                break
            

    def set_needle_pose(self, pos:List[float]=None, rpy:List[float]=None):
        """ set needle position 
        """
        assert not ((pos is None)and(rpy is None)), 'pos and rpy cannot be None in the same time'
        needle = self.client.get_obj_handle('Needle')
        if not pos is None:
            needle.set_pos(*pos)
        if not rpy is None:
            needle.set_rpy(*rpy)
        time.sleep(1) # wait for needle to move to position
        for _ in range(3):
            needle.set_force(0, 0, 0)
            needle.set_torque(0, 0, 0)
        time.sleep(0.3)
    def servo_cam_local(self, pos, quat, cam='cameraL'):
        """ servo camera w.r.t. ECM base frame 
        """
        assert cam in ['cameraL'], 'not implement'
        cam = self.client.get_obj_handle('/ambf/env/cameras/cameraL')
        rpy = Rotation.Quaternion(*quat).GetRPY()
        cam.set_pos(*pos)
        cam.set_rpy(*rpy)
        
    def get_needle_pose(self):
        needle = self.client.get_obj_handle('Needle')
        pos = needle.get_pos()
        rpy = needle.get_rpy()
        return [pos.x,pos.y,pos.z], list(rpy)
 

class PSMClient(BaseClient):
    """
    PSM crtk topics client
    """
    arm_names = ['psm1', 'psm2']
    mearsure_base_cp = None
    # grasp_point_offset = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[0,0,0.052])) # grasping point offset w.r.t. tool frame
    # grasp_point_offset = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[0,0,-0.058])) # grasping point offset w.r.t. tool frame
    # grasp_point_offset = Frame(Rotation.RPY(*[0, 0, 0]), Vector(*[0,0,0])) # grasping point offset w.r.t. tool frame


    _q_dsr =None
    _jaw_dsr = None

    is_update_marker = False
    IK_MAX_NUM = 2

    reset_jnt_psm2 = [-0.5656515955924988, -0.15630173683166504, 1.3160043954849243, -2.2147457599639893, 0.8174221515655518,-1]

    # Kp_servo_jp = 0.5
    Kp_servo_jp = 0.01
    Kv_servo_jp = 0
    Ki_servo_jp = 1
    Mag_servo_jp = 0.15
    # de_fil_ratio_servo_jp = 0.99

    def __init__(self, ros_node,
                       arm_name:str):

        super(PSMClient, self).__init__(ros_node, is_run_thread=True)
        if arm_name not in self.arm_names:
            raise Exception(f"arm name should be in {self.arm_names}")
        
        self.arm_name = arm_name
        self.grasp_point_offset = grasp_point_offset

        # ros topics

        # self.pubs["servo_tool_cp_local"] = Publisher('/CRTK/'+ arm_name +'/servo_cp', TransformStamped, queue_size=1)
        self.pubs["servo_jp"] = Publisher('/CRTK/'+ arm_name +'/servo_jp', JointState, queue_size=1)

        self.pubs["servo_jaw_jp"] = Publisher('/CRTK/'+ arm_name +'/jaw/servo_jp', JointState, queue_size=1) 


        self.subs["measured_base_cp"] = Subscriber(self.topic_wrap('/ambf/env/'+arm_name+'/baselink/State'), RigidBodyState, self._measured_base_cp_cb)
        # self.subs["measured_tool_cp_local"] = Subscriber(self.topic_wrap('/CRTK/'+arm_name+'/measured_cp'), TransformStamped, self._measured_tool_cp_local_cb)  #TODO: tool pose measure topic is not accurate and has drift, dont know why
        self.subs["measured_js"] = Subscriber(self.topic_wrap('/CRTK/'+arm_name+'/measured_js'), JointState, self._measured_js_cb)
        self.subs["is_grasp"] = Subscriber(self.topic_wrap('/CRTK/'+arm_name+'/is_grasp'), Bool, self._is_grasp_cb)
        self.ros_rate = Rate(120)  #100hz

        self.kin = PSM_KIN() # kinematics model

        self._jaw_pub_queue = Queue()
        self._arm_pub_queue = Queue()

        # self.e_prv = None
        # self.de_fil = None
        self.pids = [PID(self.Kp_servo_jp, 
                         self.Ki_servo_jp, 
                         self.Kv_servo_jp, 
                         setpoint=0, 
                         output_limits=(-self.Mag_servo_jp, self.Mag_servo_jp),
                         sample_time=0.01) for _ in range(6)]

        self.joint_calibrate_offset = np.array([0,0,0, 0,0,0])


    def reset_pose(self, q_dsr=None, walltime=None):
        if self.arm_name == 'psm2':
            _q_dsr = q_dsr or self.reset_jnt_psm2
            self.servo_jp(_q_dsr, interpolate_num=100)
            self.open_jaw()
            self.wait(walltime=walltime)
        else:
            raise NotImplementedError
    
    def wait(self, walltime=None, force_walltime=False):
        """ wait until the queues for joint angles and jaw to be zero"""
        assert not ((walltime is None) and force_walltime), 'need to assert walltime when force_walltime is True'
        start = time.time()
        while True:
            if not walltime is None:
                if (time.time()-start)>walltime:
                    self._arm_pub_queue.queue.clear()
                    self._jaw_pub_queue.queue.clear()
                    break
            if self._arm_pub_queue.empty() and self._jaw_pub_queue.empty() and (not force_walltime):
                break
            self.ros_rate.sleep()

    def servo_tool_cp_local(self, T_g_b_dsr:Frame, interpolate_num=None, clear_queue=True):
        """
        move tool to desired pose, in Roll-Pich-Yaw convension, w.r.t. base frame
        """
        T0 = self.T_g_b_dsr
        if clear_queue:
            self._arm_pub_queue.queue.clear()
        if interpolate_num is not None:
            frames = gen_interpolate_frames(T0, T_g_b_dsr, interpolate_num)
            for i, frame in enumerate(frames):
                _T_t_b_dsr = frame * self.grasp_point_offset.Inverse()
                self._arm_pub_queue.put(_T_t_b_dsr)
        else:
            _T_t_b_dsr = T_g_b_dsr * self.grasp_point_offset.Inverse()
            self._arm_pub_queue.put(_T_t_b_dsr)
        
    
    def servo_tool_cp(self,  T_g_w_dsr:Frame, interpolate_num=None, clear_queue=True):
        """
        move tool to desired pose, in Roll-Pich-Yaw convension, w.r.t. world frame
        """
        T_b_w = self.get_signal('measured_base_cp')
        T_g_b_dsr = T_b_w.Inverse()*T_g_w_dsr 
        self.servo_tool_cp_local(T_g_b_dsr, interpolate_num, clear_queue)


    def servo_jaw_jp(self, jaw_jp_dsr:float, interpolate_num=None,clear_queue=True):
        """
        move jaw joint position
        """
        if clear_queue:
            self._jaw_pub_queue.queue.clear()
        if interpolate_num is None:
            self._jaw_pub_queue.put(jaw_jp_dsr)
        else:
            qs = np.linspace(self.jaw_dsr, jaw_jp_dsr, interpolate_num).tolist()
            for q in qs:
                _jaw_jp_dsr = q
                self._jaw_pub_queue.put(_jaw_jp_dsr)

    def servo_jp(self, q_dsr, interpolate_num=None,clear_queue=True):
        q0 = self.q_dsr
        if clear_queue:
            self._arm_pub_queue.queue.clear()
        if interpolate_num is None:
            self._arm_pub_queue.put(q_dsr)
        else:
            qs = np.linspace(q0, q_dsr, interpolate_num).tolist()
            for q in qs:
                self._arm_pub_queue.put(q)
                # print(q)

    def close_jaw(self):
        self.servo_jaw_jp(0, 200)

                
    def open_jaw(self):
        self.servo_jaw_jp(0.25, 100)
        
    def get_T_g_w_from_js(self, qs):
        return self.get_signal('measured_base_cp') * SE3_2_T(self.kin.fk(qs)) * self.grasp_point_offset

    def ik_local(self, T_g_b, q0=None, ik_engine='surgical_challenge'):
        if ik_engine == 'peter_corke':
            if q0 is None:
                q0 = self.get_signal('measured_js')
            q_dsr, is_success = self.kin.ik(T_2_SE3(T_g_b), q0)
        
        elif ik_engine == 'surgical_challenge':
            q_dsr = compute_IK(T_g_b)
            is_success = True
        return q_dsr, is_success

    def ik(self, T_g_w, q0=None):
        T_b_w = self.get_signal('measured_base_cp')
        T_g_b = T_b_w.Inverse()*T_g_w 
        q_dsr, is_success = self.ik_local(T_g_b, q0)
        return q_dsr, is_success

    def run(self):
        """ publish rostopic in a fixed ros rate, like controller"""
        while not is_shutdown() and self.is_alive:

            # print(self._arm_pub_queue.qsize())
            if not self._arm_pub_queue.empty():
                data = self._arm_pub_queue.get()
                if not isinstance(data, list):
                    q0 = self.get_signal('measured_js')
                    q_dsr = None
                    for _ in range(self.IK_MAX_NUM):
                        q_dsr, is_success = self.ik_local(data, q0)
                        if not is_success:
                            q_dsr = None
                        else:
                            break
                else:
                    assert isinstance(data, list)
                    q_dsr = data

                if q_dsr is not None:
                    self._q_dsr = q_dsr
                if self._q_dsr is None:
                    self._q_dsr = self.get_signal('measured_js')
                q_msr = self.get_signal('measured_js')
                e = np.array(q_dsr) - np.array(q_msr)
                # e_prv = e if self.e_prv is None else self.e_prv
                # de = (e - e_prv)/0.01 # de = delta /dt
                # self.de_fil = de if self.de_fil is None else de * self.de_fil_ratio_servo_jp + self.de_fil * (1 - self.de_fil_ratio_servo_jp)
                # q_delta = self.Kp_servo_jp * e + self.Kv_servo_jp * self.de_fil
                # q_delta = np.clip(q_delta, -self.Mag_servo_jp, self.Mag_servo_jp)
                
                q_delta = [-self.pids[i](e[i]) for i in range(6)]
                q_delta = np.array(q_delta)
                q_dsr_servo = q_dsr + q_delta - self.joint_calibrate_offset
                msg = JointState()
                msg.position = q_dsr_servo.tolist()
                self.pubs['servo_jp'].publish(msg)
                # self.e_prv = e

                if self.is_update_marker:
                    msg = self._T2TransformStamped(self.get_signal('measured_base_cp') * SE3_2_T(self.kin.fk(q_dsr)) * self.grasp_point_offset)
                    self.pubs["servo_marker_cp"].publish(msg)


            if not self._jaw_pub_queue.empty():
                data = self._jaw_pub_queue.get()
                self._jaw_dsr = data
                msg = JointState()
                msg.position = [data] 
                self.pubs["servo_jaw_jp"].publish(msg)          

            self.ros_rate.sleep()

    @property
    def jaw_dsr(self):
        return self._jaw_dsr or 0.0 

    @property
    def q_dsr(self):
        return self._q_dsr or self.get_signal('measured_js')

    @property
    def T_g_b_dsr(self):
        """ Grasp point frame w.r.t. base, desire"""
        return SE3_2_T(self.kin.fk(self.q_dsr)) * self.grasp_point_offset
    @property
    def T_g_w_dsr(self):
        """ Grasp point frame w.r.t. world, desire"""
        return self.get_signal('measured_base_cp') * self.T_g_b_dsr  

    @property
    def T_g_b_msr(self):
        """ Grasp point frame w.r.t. base, measure"""
        _q = self.get_signal('measured_js')
        return SE3_2_T(self.kin.fk(_q)) * self.grasp_point_offset
    @property
    def T_g_w_msr(self):
        """ Grasp point frame w.r.t. world, measure"""
        return self.get_signal('measured_base_cp') * self.T_g_b_msr        

    def _measured_base_cp_cb(self, data):
        self.set_signal('measured_base_cp', self._RigidBodyState2T(data))
    
    # def _measured_tool_cp_local_cb(self, data): 
    #     _T = self._TransformStamped2T(data)
    #     self.set_signal('measured_tool_cp_local', _T)
    def _measured_js_cb(self, data):
        pos = data.position
        _pos = np.array(pos) + self.joint_calibrate_offset
        self.set_signal('measured_js', _pos.tolist())


    def _is_grasp_cb(self, data):
        self.set_signal('is_grasp', data.data)



class ECMClient(BaseClient):
    """
    ECM crtk topics client
    """
    def __init__(self, ros_node,
                       is_left_cam=True, 
                       is_right_cam=False, 
                       is_left_point_cloud=False, 
                       is_right_point_cloud=False):

        super(ECMClient, self).__init__(ros_node)

        # ros topics
        self.pubs['ecm_servo_jp'] = Publisher('/CRTK/ecm/servo_jp', JointState, queue_size=1) # we can only control joint position for ecm
        
        # self.subs['cameraL_local_state'] = Subscriber(self.topic_wrap('/ambf/env/cameras/cameraL/State'), CameraState, self._cameraL_local_state_cb)
        # self.subs['cameraR_local_state'] = Subscriber(self.topic_wrap('/ambf/env/cameras/cameraR/State'), CameraState, self._cameraR_local_state_cb)
        self.subs['camera_frame_state'] = Subscriber(self.topic_wrap('/CRTK/ecm/measured_cp'), PoseStamped, self._camera_frame_state_cb)   

        if is_left_cam:
            self.subs['cameraL_image'] = Subscriber(self.topic_wrap('/ambf/env/cameras/cameraL/ImageData'), numpy_msg(Image), self._cameraL_image_cb)
        if is_left_point_cloud:
            self.subs['cameraL_point_cloud'] = Subscriber(self.topic_wrap('/ambf/env/cameras/cameraL/DepthData'), numpy_msg(PointCloud2), self._cameraL_image_depth_cb)

        if is_right_cam:
            self.subs['cameraR_image'] = Subscriber(self.topic_wrap('/ambf/env/cameras/cameraR/ImageData'), numpy_msg(Image), self._cameraR_image_cb)

        if is_right_point_cloud:
            self.subs['cameraR_point_cloud'] = Subscriber(self.topic_wrap('/ambf/env/cameras/cameraR/DepthData'), numpy_msg(PointCloud2), self._cameraR_image_depth_cb)

        

        # TODO: there is a minor for ecm js publisher
        # self.subs['measured_cam_js'] = Subscriber('/CRTK/ecm/measured_js', JointState, self._ecm_js_cb) # there is a minor for ecm js publisher
        self.ros_rate = Rate(5)  

    def move_ecm_jp(self, pos:List[float], time_out=30, threshold=0.001, ratio=1, count_break=3):
        """
        move camera joint position, with blocking (waiting)
        """
        msg = JointState()
        msg.position = pos
        self.pubs["ecm_servo_jp"].publish(msg)

        start = time.time()
        T_prv = self.get_signal('camera_frame_state')
        cnt = 0

        # block until the camera frame
        while True:
            self.ros_rate.sleep()
            T = self.get_signal('camera_frame_state')

            if T is None or T_prv is None:
                continue


            deltaT = T_prv.Inverse()*T
            theta, _ = deltaT.M.GetRotAngle()
            dp = deltaT.p.Norm()

            if time.time() - start > time_out:
                # print("move ecm, timeout")
                break
            
            # print(dp + theta)
            if dp + theta*ratio <threshold:
                cnt+=1
            else:
                cnt = 0
            
            if cnt >=count_break:
                # print("move ecm, stop")
                break

            T_prv = T

    def _cameraL_image_cb(self, data):
        """
        ros callback
        """
        img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        self.set_signal('cameraL_image',img)

    def _cameraL_image_depth_cb(self, data):
        """
        ros callback
        """
        # img = np.frombuffer(data.data, dtype=np.uint8)
        self.set_signal('cameraL_point_cloud', data)
    
    def _cameraR_image_cb(self, data):
        """
        ros callback
        """
        img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        self.set_signal('cameraR_image',img)
    
    def _cameraR_image_depth_cb(self, data):
        """
        ros callback
        """
        # img = np.frombuffer(data.data, dtype=np.uint8)
        self.set_signal('cameraR_point_cloud', data)

    def _cameraL_local_state_cb(self, data):
        """
        ros callback
        """
        self.set_signal('cameraL_local_state',RigidBodyState2T(data))

    def _cameraR_local_state_cb(self, data):
        """
        ros callback
        """
        self.set_signal('cameraR_local_state',RigidBodyState2T(data))

    def _camera_frame_state_cb(self, data):
        """
        ros callback
        """
        self.set_signal('camera_frame_state', PoseStamped2T(data))


class SceneClient(BaseClient):
    """
    Scene crtk topics client
    """
    def __init__(self, ros_node):
        super(SceneClient, self).__init__(ros_node)
        # ros topics
        self.subs['measured_needle_cp'] = Subscriber(self.topic_wrap('/CRTK/Needle/measured_cp'), PoseStamped, self._needle_cp_cb)
        self.subs['measured_entry1_cp'] = Subscriber(self.topic_wrap('/CRTK/Entry1/measured_cp'), PoseStamped, self._measured_entry1_cp_cb)
        self.subs['measured_entry2_cp'] = Subscriber(self.topic_wrap('/CRTK/Entry2/measured_cp'), PoseStamped, self._measured_entry2_cp_cb)
        self.subs['measured_entry3_cp'] = Subscriber(self.topic_wrap('/CRTK/Entry3/measured_cp'), PoseStamped, self._measured_entry3_cp_cb)
        self.subs['measured_entry4_cp'] = Subscriber(self.topic_wrap('/CRTK/Entry4/measured_cp'), PoseStamped, self._measured_entry4_cp_cb)

        self.subs['measured_exit1_cp'] = Subscriber(self.topic_wrap('/CRTK/Exit1/measured_cp'), PoseStamped, self._measured_exit1_cp_cb)
        self.subs['measured_exit2_cp'] = Subscriber(self.topic_wrap('/CRTK/Exit2/measured_cp'), PoseStamped, self._measured_exit2_cp_cb)
        self.subs['measured_exit3_cp'] = Subscriber(self.topic_wrap('/CRTK/Exit3/measured_cp'), PoseStamped, self._measured_exit3_cp_cb)
        self.subs['measured_exit4_cp'] = Subscriber(self.topic_wrap('/CRTK/Exit4/measured_cp'), PoseStamped, self._measured_exit4_cp_cb) 

    def _needle_cp_cb(self, data):
        """
        ros callback
        """
        self.set_signal('measured_needle_cp', PoseStamped2T(data))

    def _measured_entry1_cp_cb(self, data):
        """
        ros callback
        """
        self.set_signal('measured_entry1_cp', PoseStamped2T(data))

    def _measured_entry2_cp_cb(self, data):
        """
        ros callback
        """
        self.set_signal('measured_entry2_cp', PoseStamped2T(data))
        
    def _measured_entry3_cp_cb(self, data):
        """
        ros callback
        """
        self.set_signal('measured_entry3_cp', PoseStamped2T(data))

    def _measured_entry4_cp_cb(self, data):
        """
        ros callback
        """
        self.set_signal('measured_entry4_cp', PoseStamped2T(data))


    #====
    def _measured_exit1_cp_cb(self, data):
        """
        ros callback
        """
        self.set_signal('measured_exit1_cp', PoseStamped2T(data))

    def _measured_exit2_cp_cb(self, data):
        """
        ros callback
        """
        self.set_signal('measured_exit2_cp', PoseStamped2T(data))
        
    def _measured_exit3_cp_cb(self, data):
        """
        ros callback
        """
        self.set_signal('measured_exit3_cp', PoseStamped2T(data))

    def _measured_exit4_cp_cb(self, data):
        """
        ros callback
        """
        self.set_signal('measured_exit4_cp', PoseStamped2T(data))



if __name__ == "__main__":
    # ros_node = init_node('ros_client_test',anonymous=True)
    # client = PSMClient(ros_node, 'psm1')
    # sleep(1)
    # print(client.sub_signals_names)
    # print(client.sub_signals['measured_base_cp'])



    # engine = ClientEngine()
    # engine.add_clients(['psm1', 'psm2'])
    # print(engine.client_names)
    # print(engine.get_signal('psm1', 'measured_base_cp'))
    # print(engine.get_signal('psm2', 'measured_base_cp'))

    # print("============ move ecm ")
    # engine.add_clients(['ecm'])
    # engine.clients['ecm'].servo_cam_jp([0,0,-0.9,0])

    # print(engine.get_signal('ecm','cameraL_image').shape)





    engine = ClientEngine()
    engine.add_clients(['psm2'])
    engine.start()

    q0 = [-0.5656515955924988, -0.15630173683166504, 1.3160043954849243, -2.2147457599639893, 0.8174221515655518, -1]

    sleep_time = 0.3
    engine.clients['psm2'].servo_jp(q0, interpolate_num=100)
    engine.clients['psm2'].close_jaw()
    engine.clients['psm2'].sleep(sleep_time)

    T_g_w_msr = engine.clients['psm2'].T_g_w_msr
    deltaT = RPY2T(*[0.2,0,0,0,0,0])
    engine.clients['psm2'].servo_tool_cp(deltaT * T_g_w_msr, interpolate_num=100)
    engine.clients['psm2'].open_jaw()
    engine.clients['psm2'].sleep(sleep_time)
    
    T_g_w_msr = engine.clients['psm2'].T_g_w_msr
    deltaT = RPY2T(*[0,0.2,0,0,0,0])
    engine.clients['psm2'].servo_tool_cp(deltaT * T_g_w_msr, interpolate_num=100)
    engine.clients['psm2'].close_jaw()
    engine.clients['psm2'].sleep(sleep_time)

    T_g_w_msr = engine.clients['psm2'].T_g_w_msr
    deltaT = RPY2T(*[0,0,0.1,0,0,0])
    engine.clients['psm2'].servo_tool_cp(deltaT * T_g_w_msr, interpolate_num=100)
    engine.clients['psm2'].open_jaw()
    engine.clients['psm2'].sleep(sleep_time)


    print("ctrl+c to stop")
    spin()
    engine.close()