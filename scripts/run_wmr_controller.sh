#!/usr/bin/env bash
set -e

# Lancer le contrôleur WMR (NSMC/BSMC)
source /opt/ros/humble/setup.bash
source "$(dirname "$0")/../ros2_ws/install/setup.bash"

# Paramètres par défaut (tu peux modifier ici)
CTRL_TYPE=${1:-NSMC}
TRAJ_TYPE=${2:-circle}

ros2 run wmr_controller wmr_controller \
  --ros-args \
    -p controller_type:=${CTRL_TYPE} \
    -p trajectory:=${TRAJ_TYPE} \
    -p R:=0.6 \
    -p Omega:=0.20 \
    -p Vmax:=0.22 \
    -p Wmax:=1.8 \
    -p use_disturbance:=false
