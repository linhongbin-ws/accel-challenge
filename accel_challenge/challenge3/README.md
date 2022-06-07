## Run

- Open a terminal, run
    ```sh
    roscore
    ```
- Open 2nd terminal, run the following lines to pop out simulator:
    ```sh
    cd <path to accel-challenge>
    source bash/run_simulator.sh
    ```
- Open 3rd terminal, run to start `crtk_interface`, which running controllers and rostopics
    ```sh
    cd <path to accel-challenge>
    source bash/run_crtk_interface.sh
    ```

- Open 4rd terminal to run needle tracking algorithm
    ```sh
    cd <path to accel-challenge>
    source bash/init.sh
    cd <path to accel-challenge>/accel_challenge/challenge3
    python task_completion_report_for_challenge3.py
    ```
- Open 4rd terminal to run challenge 3 trajectory script,
    ```sh
    cd <path to accel-challenge>
    source bash/init.sh
    cd <path to accel-challenge>/accel_challenge/challenge3
    python challenge3_traj.py 
    ```

- Open 6th terminal, run the evaluation for challenge3:
  ```bash
  cd <path to accel-challenge>
  source bash/init.sh
  cd  <surgical_robotics_challenge>/scripts/surgical_robotics_challenge/evaluation
  python evaluation.py -t Tstone -e 3
  ```


## Note

- This script require to use cameraL and cameraR images ros topic for calibration and tracking.


## Results

We tested our code for the case of no joint errors. It could succeed as follows:

<p align="center">
  <img src="/accel_challenge/challenge3/media/sucess_without_joint_error.png" width="700" title="suceed without joint error">
</p>

We have a video for sucessfully finishing the task 3 under the no-joint-error case. Here are the videos

- [case 1](https://mycuhk-my.sharepoint.com/:v:/g/personal/1155135739_link_cuhk_edu_hk/EQ5RWHKIelhEmrkmoWjs-s0BvzU_CM2GonIs82xWp5jY3w?e=CKPaGB)

- [case 2](https://mycuhk-my.sharepoint.com/:v:/g/personal/1155135739_link_cuhk_edu_hk/ETHtVZcRWlJPqk4J2ge2POMBcmceeFsfaVF0wt4rih8lYw?e=MyXMgA)
- [case 3](https://mycuhk-my.sharepoint.com/:v:/g/personal/1155135739_link_cuhk_edu_hk/EU4yhznuLa1Pr3MJ3ycrt7IBbz8XnLL_cSnGFdgA2NpTCA?e=9icUln)

For the error cases, we have calibration procedures to predict joint error. But due to the time limit, we do no have chances to test it.





