# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:noetic-ros-base

# Source ROS
RUN apt-get update && apt-get install -y ros-noetic-tf
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc

# Install Git, Python pip, tf
RUN apt-get update && apt-get install -y git python3-pykdl pip

# clone AMBF Msgs and Python Client
RUN cd  && git clone https://github.com/adnanmunawar/ambf_python_client.git
RUN . /opt/ros/noetic/setup.sh && cd ~/ambf_python_client && mkdir build && cd build && cmake .. && make
RUN echo 'source ~/ambf_python_client/build/devel/setup.bash' >> ~/.bashrc

# clone surgical_robotics_challenge
RUN cd && git clone https://github.com/collaborative-robotics/surgical_robotics_challenge

# Install the scripts
RUN cd && cd surgical_robotics_challenge/scripts/ && pip install -e .


#refer to https://stackoverflow.com/questions/65492490/how-to-conda-install-cuda-enabled-pytorch-in-a-docker-container
# install pytorch with cuda
RUN pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu113

# RUN apt-get install -y wget bzip2 ca-certificates libglib2.0-0 libxext6 libsm6 libxrender1 git mercurial subversion && \
#         apt-get clean
# RUN wget --quiet https://repo.anaconda.com/archive/Anaconda3-2020.02-Linux-x86_64.sh -O ~/anaconda.sh && \
#         /bin/bash ~/anaconda.sh -b -p /opt/conda && \
#         rm ~/anaconda.sh && \
#         ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
#         echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc && \
#         find /opt/conda/ -follow -type f -name '*.a' -delete && \
#         find /opt/conda/ -follow -type f -name '*.js.map' -delete && \
#         /opt/conda/bin/conda clean -afy
# # set path to conda
# ENV PATH /opt/conda/bin:$PATH

# RUN conda update conda \
#     && conda create -n accel_challenge python=3.7 


# # PyTorch with CUDA 11.3
# RUN conda install -y -n accel_challenge pytorch torchvision torchaudio cudatoolkit=11.3 -c pytorch

# # change default shell to bash
# SHELL ["/bin/bash", "-c"] 
# COPY ./requirements.txt /root/requirements.txt


RUN pip3 install -U pip && pip3 install tensorflow==2.8 protobuf==3.20.0
COPY ./requirements_docker.txt /root/requirements.txt
RUN pip3 install -r /root/requirements.txt

RUN apt-get install -y wget
RUN wget --quiet https://repo.anaconda.com/archive/Anaconda3-2020.02-Linux-x86_64.sh -O ~/anaconda.sh && \
        /bin/bash ~/anaconda.sh -b -p /opt/conda && \
        rm ~/anaconda.sh && \
        ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
        echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc && \
        find /opt/conda/ -follow -type f -name '*.a' -delete && \
        find /opt/conda/ -follow -type f -name '*.js.map' -delete && \
        /opt/conda/bin/conda clean -afy
# set path to conda
ENV PATH /opt/conda/bin:$PATH
RUN conda create -n accel_challenge python=3.8 
RUN conda install -y -n accel_challenge cudatoolkit=11.3 -c pytorch
RUN conda install -y -n accel_challenge cudnn -c conda-forge
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/conda/envs/accel_challenge/lib/" >> ~/.bashrc