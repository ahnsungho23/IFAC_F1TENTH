#!/usr/bin/env zsh
set -e

readonly REPO=/home/sungho/Documents/GitHub/2026_IFAC
readonly TOOLS=$REPO/planning_study/gqsc_s1_live_runtime_qualification_v1/tools
readonly PACKAGE=$TOOLS/gqsc_runtime_replay

source /opt/ros/humble/setup.zsh
source /home/sungho/sim_ws/install/setup.zsh
source $REPO/install/setup.zsh

colcon --log-base $TOOLS/_log build \
  --base-paths $PACKAGE \
  --build-base $TOOLS/_build \
  --install-base $TOOLS/_install \
  --merge-install \
  --packages-select gqsc_runtime_replay \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

