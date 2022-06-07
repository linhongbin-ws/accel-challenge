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

- Open 4rd terminal to run challenge scripts,
    run challenge#2 
    ```sh
    cd <path to accel-challenge>
    source bash/init.sh
    cd <path to accel-challenge>/accel_challenge/challenge3
    python task_completion_report_for_challenge3.py
    ```
- Open 4rd terminal to run challenge scripts,
    run challenge#2 
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

- This script require to use cameraL image ros topic for calibration.

