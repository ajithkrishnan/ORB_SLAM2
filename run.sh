#!/bin/bash

#DATA_PATH="/share/projects/2019_kss_personal_jupyter_notebooks/trn_ak/sts_odom_dataset/paketzentrum_eifeltor/sequences/2019-05-15-14-55-45_hmm"
DATA_PATH="/share/projects/2019_kss_personal_jupyter_notebooks/trn_ak/sts_odom_dataset/paketzentrum_eifeltor/eval_bags/*"


for dir in $DATA_PATH     # list directories in the form "/tmp/dirname/"
do
    #dir=${dir%*/}      # remove the trailing "/"
    #echo ${dir##*/}    # print everything after the final "/"
    echo "reading from "$dir
    ./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/sts.yaml $dir
done

#./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/sts.yaml $DATA_PATH

