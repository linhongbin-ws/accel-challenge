# Overview
This is our team (Tstone) task1 code repository which is inherited from the official library.

https://github.com/collaborative-robotics/surgical_robotics_challenge

To simplify the code structure as much as possible, we only modify the code scripts/surgical_robotics_challenge/task_completion_report.py and turn on "publish image" option in world_stereo.yaml, so it is very easy to see our computational process clearly.

After the basic installation, you can run it directly and we will publish the estimation results with the official API.
You can acquire our estimation error by running scripts/surgical_robotics_challenge/evaluation/evaluation.py.

As a reference, we test our algorithm and achieve Task 1 Overall Score with **0.002 ~ 0.015 Simulation Units** with correct setup~

*Our platform: Intel® Core™ i9-10900KF CPU + GeForce RTX 3090*

*Usage: cost ~6G GPU Room*

*Time cost: 30s ~ 55s depending on the distance (we ensure the best performance within 1 min, although the iteration process can be terminated in advance.)*

Therefore please feel free to contact us (Name: Bin LI, Email: [bli@mae.cuhk.edu.hk](bli@mae.cuhk.edu.hk)) if you have any questions in estimating our algorithm. Hope you can enjoy it ~


# Get started
- To startup the environment, open a terminal, run
    ```bash
    roscore
    ```
- Open 2nd terminal, run the following lines in ambf folder (ambf/bin/lin-x86_64) to pop out the simulator:
  ```bash
  ./ambf_simulator --launch_file <surgical_robotics_challenge>/launch.yaml -l 0,1,3,4,14,15 -p 200 -t 1 --override_max_comm_freq 120
  ```
- Open 3rd terminal, run the needle pose estimation:
  ```bash
  conda activate accel_challenge
  cd <accel-challenge>/accel_challenge/challenge1
  python challenge1.py -t Tstone -e 1
  ```
  If there exists cv_bridge errors, please run the following code:
  ```bash
  conda activate accel_challenge
  source ~/catkin_ws/devel/setup.bash (Note that we build the python3 version cv_bridge in ~/catkin_ws folder)
  cd <accel-challenge>/accel_challenge/challenge1
  python challenge1.py -t Tstone -e 1
  ```
- Open 4th terminal, run the evaluation for our estimated pose estimation:
  ```bash
  conda activate accel_challenge
  cd  <surgical_robotics_challenge>/scripts/surgical_robotics_challenge/evaluation
  python evaluation.py -t Tstone -e 1
  ```

# Notes for organizers
As a reference, the local Task 1 Overall Score of our method is between **0.002 ~ 0.015 Simulation Units**.
If the result of your test deviates from this, there may be a problem with the relevant settings, we also have alternative solutions in the source code (already annotated), please feel free to contact us (Name: Bin LI, Email: bli@mae.cuhk.edu.hk), thank you!

[comment]: <> (-------------------------------------------------------------------------------------------------)

[comment]: <> (# Surgical Robotics Challenge)

[comment]: <> (For more information regarding the challenge, please visit https://collaborative-robotics.github.io/surgical-robotics-challenge/challenge-2021.html)

[comment]: <> (# [Discussions Forum]&#40;https://github.com/collaborative-robotics/surgical_robotics_challenge/discussions&#41;)

[comment]: <> (Please checkout the [Discussions Tab]&#40;https://github.com/collaborative-robotics/surgical_robotics_challenge/discussions&#41; for asking questions, posting suggestions, connecting with the community and for keeping up to date with the challenge.)

[comment]: <> (# 1. Install AMBF and ROS Prerequisites)

[comment]: <> (Make sure that the correct version of ROS is installed and sourced on your system. For streaming the image and depth data out of AMBF, please also install the following ROS packages)

[comment]: <> (- cv_bridge)

[comment]: <> (- image_transport)

[comment]: <> (```bash)

[comment]: <> (apt-get install ros-<version>-cv-bridge ros-<version>-image-transport)

[comment]: <> (```)

[comment]: <> (Then, clone, build and source AMBF's `ambf-2.0` branch.)


[comment]: <> (https://github.com/WPI-AIM/ambf/tree/ambf-2.0)

[comment]: <> (First time cloning:)

[comment]: <> (```bash)

[comment]: <> (git clone https://github.com/WPI-AIM/ambf.git)

[comment]: <> (cd ambf)

[comment]: <> (git checkout -b ambf-2.0 origin/ambf-2.0)

[comment]: <> (```)

[comment]: <> (In case there are updates to AMBF, you can simply update your local copy by:)

[comment]: <> (```bash)

[comment]: <> (git pull)

[comment]: <> (```)

[comment]: <> (Don't forget to build the repo using the instructions on AMBF's Readme)

[comment]: <> (# 2. Clone this repo to your local machine OR use a Dockerfile)

[comment]: <> (#### Option 1: &#40;Clone repo to your local machine&#41;)

[comment]: <> (  Please refer to [README]&#40;./scripts/README.md&#41; in the [scripts]&#40;./scripts&#41; folder for instructions on installing the Python package for system-wide access.)

[comment]: <> (#### Option 2: &#40;Use Dockerfile&#41;)

[comment]: <> (  You can alternatively use Dockerfiles to create Docker images by following the instructions here:)

[comment]: <> (  https://github.com/collaborative-robotics/docker_surgical_robotics_challenge)


[comment]: <> (# 3. Running the simulation)

[comment]: <> (  The simulation is spawned in AMBF with the launch file and AMBF Description Format &#40;ADF&#41; files from this repo:)

[comment]: <> (  The `ambf_simulator` binary resides in `ambf/bin/lin-x86_64`. You should be in that directory before running the commands below. Alternatively, you can create a symlink to this binary.)

[comment]: <> (  ```bash)

[comment]: <> (  ./ambf_simulator --launch_file <surgical_robotics_challenge>/launch.yaml -l 0,1,3,4,14,15 -p 120 -t 1 --override_max_comm_freq 120)

[comment]: <> (  ```)

[comment]: <> (  This is an example of what the scene should look like &#40;minus the motions of the PSM, Needle etc.&#41;:)

[comment]: <> (  <p align="center">)

[comment]: <> (  <img src=Media/figure_eight.gif width="480"/>)

[comment]: <> (  </p>)

[comment]: <> (  To launch a different scene with just the needle &#40;without any thread&#41;, you can run:)

[comment]: <> (  ```bash)

[comment]: <> (  ./ambf_simulator --launch_file <surgical_robotics_challenge>/launch.yaml -l 0,1,3,4,13,14 -p 200 -t 1 --override_max_comm_freq 120)

[comment]: <> (  ```)

[comment]: <> (  And this is what the scene should now look like:)

[comment]: <> (  <p align="center">)

[comment]: <> (  <img src=Media/neede_without_thread.gif width="480"/>)

[comment]: <> (  </p>)


[comment]: <> (### 3a. The launch file:)

[comment]: <> (  To understand the launch file, visit the following link:)

[comment]: <> (  https://github.com/WPI-AIM/ambf/wiki/Selecting-Robots)

[comment]: <> (### 3b. Simulated Cameras)

[comment]: <> (  The simulated camera&#40;s&#41; is defined in the World file &#40;[`world_stereo.yaml`]&#40;./ADF/world/world_stereo.yaml&#41;&#41; which is set in the [`launch.yaml`]&#40;./launch.yaml&#41; file.)

[comment]: <> (  To enable the camera&#40;s&#41; to publish the scene image or depth data, follow the instructions on this page:)

[comment]: <> (  https://github.com/WPI-AIM/ambf/wiki/Camera-feed-and-depth-camera)

[comment]: <> (### 3c. Camera Coordinate frames)

[comment]: <> (  To better understand the different camera coordinate frames and the difference between the AMBF and the Opencv camera convention, please refer to [camera_convention.md]&#40;./docs/camera_conventions.md&#41;)

[comment]: <> (### 3c. Resetting the Simulation)

[comment]: <> (  You can press `CTRL+R` to reset the rigid bodies in simulation, and `CTRL+V` to reset the camera pose if you changed it with the mouse.)

[comment]: <> (### 3d. Launch Arguments:)

[comment]: <> (  The launch arguments provided above e.g. &#40;`-l 0,1,3,4,14,15 -p 200 -t 1`&#41; define the launch file, the list of ADF files to load, simulation frequency and time-stepping technique. For a full list of arguments, please refer to this link:)

[comment]: <> (  https://github.com/WPI-AIM/ambf/wiki/Command-Line-Arguments)


[comment]: <> (# 4. Interacting with Simulated Robots using Python Scripts:)

[comment]: <> (Please take a look at the scripts in the [`scripts`]&#40;./scripts&#41; folder:)


[comment]: <> (# 5. Controlling via Input Devices)

[comment]: <> (The code in the scripts folder allows the dVRK MTMs or Geomagic Touch / Phantom Omni to control the simulated PSMs.)

[comment]: <> (With the simulation already running, run the `dvrk-ros` application for the `dVRK MTMs` or the ROS application for the `Geomagic Touch/Phantom Omni`. You can find the relevant code for them here:)

[comment]: <> (**a. https://github.com/jhu-dvrk/dvrk-ros** &#40;dvrk-ros&#41;)

[comment]: <> (**b. https://github.com/WPI-AIM/ros_geomagic** &#40;geomagic_touch/phantom_omni&#41;)

[comment]: <> (Then run one of the corresponding python scripts:)

[comment]: <> (**a. scripts/surgical_robotics_challenge/teleoperation/mtm_multi_psm_control.py** &#40;For MTMs&#41;)

[comment]: <> (**b. scripts/surgical_robotics_challenge/geomagic_multi_psm_control.py** &#40;For Geomagic Touch/Phantom Omni&#41;)

[comment]: <> (Refer to the `README` in the scripts folder for further information)

[comment]: <> (# 6. Citation)

[comment]: <> (If you find this work useful, please cite it as:)

[comment]: <> (```bibtex)

[comment]: <> (@article{munawar2022open,)

[comment]: <> (  title={Open Simulation Environment for Learning and Practice of Robot-Assisted Surgical Suturing},)

[comment]: <> (  author={Munawar, Adnan and Wu, Jie Ying and Fischer, Gregory S and Taylor, Russell H and Kazanzides, Peter},)

[comment]: <> (  journal={IEEE Robotics and Automation Letters},)

[comment]: <> (  volume={7},)

[comment]: <> (  number={2},)

[comment]: <> (  pages={3843--3850},)

[comment]: <> (  year={2022},)

[comment]: <> (  publisher={IEEE})

[comment]: <> (})

[comment]: <> (```)
