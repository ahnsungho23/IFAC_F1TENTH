#!/usr/bin/env zsh
set -e

readonly REPO=/home/sungho/Documents/GitHub/2026_IFAC
readonly ROOT=$REPO/planning_study/gqsc_s1_live_runtime_qualification_v1
readonly OVERLAY=$ROOT/release_overlay
readonly BUILD=$OVERLAY/_build
readonly INSTALL=$OVERLAY/_install
readonly LOG=$OVERLAY/_log
readonly SOURCE_COMMIT=dc33b875a938fdb7730d35fe61de43ee863b04c0

[[ $(git -C $REPO branch --show-current) == research ]] || exit 65
git -C $REPO diff --quiet $SOURCE_COMMIT -- src/local_planning || exit 65
print -r -- '21e653cf9b063c9c60843c6ebaeb06fda918046bfe6d78a7d4ba6b957bdddd40  '$REPO/src/local_planning/include/local_planning/gqsc_s1_frozen_contract.hpp |
  sha256sum --check --status || exit 65
print -r -- '4fe480351a80135ff2a6e4592f661ff8a5670c032e12554d85065339d16ea960  '$REPO/src/local_planning/config/local_planning.yaml |
  sha256sum --check --status || exit 65
print -r -- '572adb59ea24f3f06bed7502eb870e57a33a5416e8630106b67bc2b1d0b405bf  '$REPO/planning_study/p3_geometry_conditioned_method_v1/success_control_inputs/SCE018.event |
  sha256sum --check --status || exit 65

for path in $BUILD $INSTALL $LOG; do
  [[ ! -e $path ]] || { print -u2 -- "refusing to overwrite $path"; exit 73; }
done

source /opt/ros/humble/setup.zsh
source /home/sungho/sim_ws/install/setup.zsh
source $REPO/install/setup.zsh

colcon --log-base $LOG build \
  --base-paths $REPO/src/local_planning \
  --build-base $BUILD \
  --install-base $INSTALL \
  --packages-select local_planning \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DBUILD_TESTING=ON
