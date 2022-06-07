## Dowload

- open a terminal

    ```sh
    cd
    git clone https://github.com/linhongbin-ws/accel-challenge.git
    cd <path to accel-challenge>
    git clone https://github.com/collaborative-robotics/surgical_robotics_challenge.git
    ```

- Since our trained model is large, we put all trained model on the cloud. Please download them to merge to the directory  `<path to accel-challenge>/model/`

## Install

- Install [ambf](https://github.com/WPI-AIM/ambf)
- create conda virtual environment python=3.7 . 
    ```sh
    conda create -n accel_challenge python=3.7
    conda activate accel_challenge
    pip install torch==1.8.2 torchvision==0.9.2 torchaudio==0.8.2 --extra-index-url https://download.pytorch.org/whl/lts/1.8/cu111
    python -m pip install detectron2 -f https://dl.fbaipublicfiles.com/detectron2/wheels/cu111/torch1.8/index.html  
    pip install scikit-image
    pip install -r requirements.txt
    ```
- Install PyKDL on virtual environment from source, follow the [instruction](https://blog.csdn.net/qq_42237662/article/details/109783935)

    (note: make sure to uninstall the ros-kdl packages in the system before install PyKDL:
   ```sh
   sudo find / -iname PyKDL.so # this will print out all paths to PyKDL.so
   sudo rm -rf <path to>/PyKDL.so
   ```


- Install surgical_robot_challenge
    ```sh
    cd <path to surgical_robotics_challenge>/scripts/
    pip install -e .
    ```
- Install accel-challenge
    ```sh
    cd <path to accel-challenge>
    conda install -c conda-forge wxpython # deeplabcut depends on this package
    pip install -r requirements.txt
    pip install -e .
    ```  

- modify a minor in original surgical_robotics_challenge, edit file `<path to surgical_robotics_challenge>/scripts/surgical_robotics_challenge/launch_crtk_interface.py`
    from 
    ```py
    import psm_arm
    import ecm_arm
    import scene
    ```
    to
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


- modify the file `<path to accel-challenge>/accel_challenge/bash/user_var.sh`, there are several path variables. You need to modify according to your enviroment path.
    ```sh
    AMBF_PATH="/home/ben/ssd/code/robot/ambf"
    SURGICAL_CHALLENGE_PATH='/home/ben/ssd/code/robot/accel-challenge/surgical_robotics_challenge'
    ANACONDA_PATH="/home/ben/anaconda3" 
    ENV_NAME="accel_challenge" # conda virtual environment name
    ```


## How to Run

- To run Challenge #1, please refer to [README](https://github.com/linhongbin-ws/accel-challenge/tree/master/accel_challenge/challenge1)
- To run Challenge #2, please refer to [README](https://github.com/linhongbin-ws/accel-challenge/tree/master/accel_challenge/challenge2)
- To run Challenge #3, please refer to [README](https://github.com/linhongbin-ws/accel-challenge/tree/master/accel_challenge/challenge3)