source ./bash/init.sh
# echo $AMBF_PATH
echo $SURGICAL_CHALLENGE_PATH
pushd $SURGICAL_CHALLENGE_PATH
# $AMBF_PATH/bin/lin-x86_64/ambf_simulator --launch_file ./launch.yaml -l 0,1,2,3,4,5,14,15 -p 200 -t 1 # full setup
# $AMBF_PATH/bin/lin-x86_64/ambf_simulator --launch_file ./launch_modified.yaml -l 3,4,13, 14 -p 200 -t 1 # just psm2 and simple stuff
$AMBF_PATH/bin/lin-x86_64/ambf_simulator --launch_file ./launch.yaml -l 0,1,3,4,14,15 -p 120 -t 1 --override_max_comm_freq 120
popd