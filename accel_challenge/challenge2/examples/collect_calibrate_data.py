from accel_challenge.challenge2.ros_client import ClientEngine
from rospy import spin, Publisher, init_node, Rate
from time import sleep
from geometry_msgs.msg import PoseStamped
from accel_challenge.challenge2.tool import RPY2T, T2PoseStamped
from std_msgs.msg import Bool



#======init variables
engine =  ClientEngine()
engine.add_clients(['ecm','psm2','scene'])
engine.start()



def cam_render_test():
    """ render to check image render quality
    """
    import cv2 
    print("tab ``q`` to exit..")
    engine.clients['psm2'].reset_pose()
    engine.clients['psm2'].wait()
    q_dsr = engine.clients['psm2'].q_dsr
    # q_dsr[3] -=0.5
    is_move =False
    while(True):
        # Capture frame-by-frame
        frame = engine.get_signal('ecm','cameraL_image')
        # Display the resulting frame
        cv2.imshow('preview',frame)
        if not is_move:
            q_dsr[2] +=0.2
            engine.clients['psm2'].servo_jp(q_dsr, interpolate_num=600,clear_queue=False)
            q_dsr[1] +=0.2
            engine.clients['psm2'].servo_jp(q_dsr, interpolate_num=600,clear_queue=False)
            is_move=True
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()

cam_render_test()
# print("ctrl c to stop")
# spin()
engine.close()