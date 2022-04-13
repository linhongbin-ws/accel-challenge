source bash/user_var.sh
source $ANACONDA_PATH/bin/activate
conda activate $ENV_NAME
source $AMBF_PATH/build/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3/dist-packages/
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ben/anaconda3/envs/ambf3/lib/
