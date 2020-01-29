#!/bin/bash

port=4333

. /share/projects/2019_kss_persistent_jupyter_on_gpu1/ws/devel/setup.bash ; pwd; ls;  jupyter notebook --NotebookApp.token='al_token' --notebook-dir=/share/projects/2019_kss_personal_jupyter_notebooks/trn_ak/ --ip 0.0.0.0 --port $port --allow-root
