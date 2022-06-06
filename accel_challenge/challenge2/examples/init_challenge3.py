from accel_challenge.challenge2.examples.calibrate import set_error, calibrate_joint_error
import time


#===self testing (comment when evaluation)
offset = [0,0,0, 0,0,0]
set_error('psm1', offset)
set_error('psm2', offset)
# time.sleep(2)