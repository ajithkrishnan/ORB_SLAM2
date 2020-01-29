#!/bin/bash

#DATA_PATH="/share/projects/2019_kss_personal_jupyter_notebooks/trn_ak/sts_odom_dataset/paketzentrum_eifeltor/sequences/"
DATA_PATH="/share/projects/2019_kss_personal_jupyter_notebooks/trn_ak/sts_odom_dataset/paketzentrum_eifeltor/sequences/2019-05-15-14-55-45_hmm"

./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/sts.yaml $DATA_PATH



