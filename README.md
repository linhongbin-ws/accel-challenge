## Dowload

- open a terminal

    ```sh
    git clone https://github.com/linhongbin-ws/accel-challenge.git
    ```

## Install

- Install [ambf](https://github.com/WPI-AIM/ambf)
- create conda virtual environment python=3
    ```sh
    conda create -n accel_challenge python=3.7
    conda activate accel_challenge
    conda install pytorch torchvision torchaudio cudatoolkit=11.3 -c pytorch # if you have gpu, other options can follow pytorch official website
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
    cd <path to surgical_robotics_challenge>/scripts/
    pip install -e .
    sudo apt-get install ros-noetic-ros-numpy
    ```
- Install accel-challenge
    ```
    cd <path to gym-suture>
    pip install -e .
    ```  

## Run

- First modify the file `<package_path>\bash\user_var.sh`, there are several path variables. You need to modify according to your enviroment path.
    ```sh
    AMBF_PATH="/home/ben/code/robot/ambf"
    ANACONDA_PATH="/home/ben/anaconda3" 
    ENV_NAME="gym_suture" # conda virtual environment name
    ```
- Open a terminal, run
    ```sh
    roscore
    ```
- Open 2nd terminal, run the following lines to pop out simulator:
    ```sh
    cd <path to gym-suture>
    source bash/run_simulator.sh
    ```
- Open 3rd terminal, run to start `crtk_interface`, which running controllers and rostopics
    ```sh
    cd <path to gym-suture>
    source bash/run_crtk_interface.sh
    ```
  Now you have finished startup. Next you can run gym environment or script control. For example:
    ```sh
    cd <path to gym-suture>
    python gym_suture/scripts/challenge2_traj.py 
    # scipt Challenge2(needle picking & insertion) demo
    ```

    ```sh
    cd <path to gym-suture>
    python gym_suture/envs/wrapper.py # gym enviroment demo
    ```


## reference 

- https://gym.openai.com/docs/
- https://github.com/openai/gym/tree/master/gym/spaces