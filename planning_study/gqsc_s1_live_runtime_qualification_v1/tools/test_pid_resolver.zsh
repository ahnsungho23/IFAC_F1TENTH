#!/usr/bin/env zsh
set -e
setopt TYPESET_SILENT

readonly REPO=/home/sungho/Documents/GitHub/2026_IFAC
readonly TOOLS=$REPO/planning_study/gqsc_s1_live_runtime_qualification_v1/tools
readonly TEST_ROOT=$(mktemp -d /tmp/gqsc_s1_pid_resolver_test.XXXXXX)
typeset -a test_pids

cleanup() {
  local pid temp_file
  for pid in $test_pids; do
    kill -TERM $pid 2>/dev/null || true
    wait $pid 2>/dev/null || true
  done
  for temp_file in $TEST_ROOT/*(N); do
    rm -f -- $temp_file
  done
  rmdir -- $TEST_ROOT 2>/dev/null || true
}
trap cleanup EXIT INT TERM

source $TOOLS/pid_evidence.zsh

/usr/bin/sleep 30 &
test_pids+=($!)
typeset single_pid=$test_pids[-1]
resolve_unique_child_executable $$ /usr/bin/sleep 20 \
  >$TEST_ROOT/single.stdout 2>$TEST_ROOT/single.stderr
[[ ! -s $TEST_ROOT/single.stderr ]]
[[ $(wc -l <$TEST_ROOT/single.stdout) == 1 ]]
typeset single_output=$(<$TEST_ROOT/single.stdout)
[[ $single_output == <-> && $single_output == $single_pid ]]
verify_resolved_pid $single_output /usr/bin/sleep \
  >$TEST_ROOT/verify.stdout 2>$TEST_ROOT/verify.stderr
[[ ! -s $TEST_ROOT/verify.stdout && ! -s $TEST_ROOT/verify.stderr ]]
kill -TERM $single_pid
wait $single_pid 2>/dev/null || true
test_pids=()

/usr/bin/sleep 30 &
test_pids+=($!)
/usr/bin/sleep 30 &
test_pids+=($!)
if resolve_unique_child_executable $$ /usr/bin/sleep 2 \
    >$TEST_ROOT/ambiguous.stdout 2>$TEST_ROOT/ambiguous.stderr; then
  print -u2 -- "ambiguous resolver self-test unexpectedly succeeded"
  exit 1
fi
[[ ! -s $TEST_ROOT/ambiguous.stdout && -s $TEST_ROOT/ambiguous.stderr ]]
grep -Fq 'ambiguous child resolver' $TEST_ROOT/ambiguous.stderr
typeset pid
for pid in $test_pids; do
  kill -TERM $pid
  wait $pid 2>/dev/null || true
done
test_pids=()

if resolve_unique_child_executable $$ /usr/bin/false 2 \
    >$TEST_ROOT/zero.stdout 2>$TEST_ROOT/zero.stderr; then
  print -u2 -- "zero-candidate resolver self-test unexpectedly succeeded"
  exit 1
fi
[[ ! -s $TEST_ROOT/zero.stdout && -s $TEST_ROOT/zero.stderr ]]
grep -Fq 'zero matching PIDs' $TEST_ROOT/zero.stderr

print PID_RESOLVER_SELF_TEST_PASS
