#!/usr/bin/env zsh
set -e

if (( $# != 1 )); then
  print -u2 -- "usage: $0 FROZEN_TREEISH"
  exit 64
fi

readonly TREEISH=$1
readonly REPO=/home/sungho/Documents/GitHub/2026_IFAC
readonly ROOT_REL=planning_study/gqsc_s1_live_runtime_qualification_v1
readonly ROOT=$REPO/$ROOT_REL

git -C $REPO cat-file -e "$TREEISH^{tree}" 2>/dev/null || {
  print -u2 -- "missing frozen Git tree: $TREEISH"
  exit 65
}

typeset -a DIRECT_SCRIPTS=(
  tools/build_tools.zsh
  tools/build_release_overlay.zsh
  tools/preflight_executable_modes.zsh
  tools/run_w1_qualification.zsh
  tools/run_w2_qualification.zsh
  tools/run_w3_qualification.zsh
  tools/run_w2_smoke.zsh
  tools/run_w3_smoke.zsh
)

typeset relative_path full_path tree_mode
for relative_path in $DIRECT_SCRIPTS; do
  full_path=$ROOT/$relative_path
  if [[ ! -f $full_path ]]; then
    print -u2 -- "missing directly executed script: $relative_path"
    exit 66
  fi
  if [[ ! -x $full_path ]]; then
    print -u2 -- "filesystem executable bit missing: $relative_path"
    exit 67
  fi
  tree_mode=$(git -C $REPO ls-tree $TREEISH -- $ROOT_REL/$relative_path | awk 'NR == 1 {print $1}')
  if [[ $tree_mode != 100755 ]]; then
    print -u2 -- "frozen tree mode is ${tree_mode:-MISSING}, expected 100755: $relative_path"
    exit 68
  fi
done

print -- "EXECUTABLE_MODE_PREFLIGHT_PASS"
print -- "treeish=$TREEISH"
print -- "checked=${#DIRECT_SCRIPTS}"
