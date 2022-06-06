## Dowload

- open a terminal

    ```sh
    git clone https://github.com/linhongbin-ws/accel-challenge.git
    ```

## Install

- Install [ambf](https://github.com/WPI-AIM/ambf)
- create conda virtual environment python=3. 
    ```sh
    conda create -n accel_challenge python=3.7
    conda activate accel_challenge
    conda install pytorch cudatoolkit=11.3 -c pytorch # if you have gpu, other options can follow pytorch official website
    pip install -r requirements.txt
    ```
- Install PyKDL on virtual environment, follow the [instruction](https://blog.csdn.net/qq_42237662/article/details/109783935)

    (note: make sure to uninstall the ros-kdl packages in the system before install PyKDL:
   ```sh
   sudo find / -iname PyKDL.so # this will print out all paths to PyKDL.so
   sudo rm -rf <path to>/PyKDL.so
   ```


- Install surgical_robot_challenge
    ```
    git clone https://github.com/collaborative-robotics/surgical_robotics_challenge
    cd <path to accel-challenge>surgical_robotics_challenge/scripts/
    pip install -e .
    ```
- Install accel-challenge
    ```
    cd <path to accel-challenge>
    pip install -e .
    ```  

- modify a minor in original surgical_robotics_challenge, edit 
    from 
    ```py
    import psm_arm
    import ecm_arm
    import scene
    ```
    ```py
    from surgical_robotics_challenge import psm_arm
    from surgical_robotics_challenge import ecm_arm
    from surgical_robotics_challenge import scene
    ```

<!-- - install GPU support for DLC (optional)
  ```sh
  conda install -c conda-forge cudnn=8.2 cudatoolkit=11.3 # for tensorflow 2.8
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ben/anaconda3/envs/accel_challenge/lib/ # everytime for init
  ``` -->

## Run

- First modify the file `<path to accel-challenge>/accel_challenge/challenge2/eval/user_var.sh`, there are several path variables. You need to modify according to your enviroment path.
    ```sh
    AMBF_PATH="/home/ben/ssd/code/robot/ambf"
    SURGICAL_CHALLENGE_PATH='/home/ben/ssd/code/robot/accel-challenge/surgical_robotics_challenge'
    ANACONDA_PATH="/home/ben/anaconda3" 
    ENV_NAME="accel_challenge" # conda virtual environment name
    ```
- Open a terminal, run
    ```sh
    roscore
    ```
- Open 2nd terminal, run the following lines to pop out simulator:
    ```sh
    cd <path to accel-challenge>/accel_challenge/challenge2
    source eval/run_simulator.sh
    ```
- Open 3rd terminal, run to start `crtk_interface`, which running controllers and rostopics
    ```sh
    cd <path to accel-challenge>/accel_challenge/challenge2
    source eval/run_crtk_interface.sh
    ```

- Open 4rd terminal to run challenge scripts,
    run challenge#2 
    ```sh
    cd <path to accel-challenge>/accel_challenge/challenge2
    python example/challenge2_traj.py 
    ```
    or

    run challenge#3 
    ```sh
    cd <path to accel-challenge>/accel_challenge/challenge2
    python example/challenge3_traj.py 
    ```

