from accel_challenge.challenge2.examples.calibrate import set_error, joint_error_test, ERROR_MAG_ARR, DLC_CONFIG_PATH, ERROR_DATA_DIR, ONLINE_TEST_TRAJ_SEED
from pathlib import Path
import numpy as np
from accel_challenge.challenge2.ros_client import ClientEngine


TEST_SEED = 44


engine =  ClientEngine()
engine.add_clients(['ecm','psm2'])
engine.start()

rng_error = np.random.RandomState(TEST_SEED) # use local seed
_error = rng_error.uniform(-ERROR_MAG_ARR, ERROR_MAG_ARR)

set_error('psm2', _error)
load_dict = {'dlc_config_path':DLC_CONFIG_PATH,
            'keras_model_path':str(Path(ERROR_DATA_DIR) / 'model.hdf5'),
            'scalers_path':str(Path(ERROR_DATA_DIR) / 'scalers.pkl')}
error_pred = joint_error_test(seed=ONLINE_TEST_TRAJ_SEED,  _engine=engine,
                            load_dict=load_dict, is_predict=True, arm_name='psm2')

print(f"predict error: {error_pred}")
print(f"actual error: {_error}")
engine.close()



