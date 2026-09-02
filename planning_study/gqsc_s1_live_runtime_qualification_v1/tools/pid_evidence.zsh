# Source-only PID identity helpers for runtime qualification tools.

_pid_matches_expected_process_quiet() {
  emulate -L zsh
  setopt TYPESET_SILENT
  local pid=$1
  local expected_executable=$2
  local actual_executable argv0

  [[ $pid == <-> && -d /proc/$pid ]] || return 1
  actual_executable=$(readlink -f /proc/$pid/exe 2>/dev/null) || return 1
  [[ $actual_executable == $expected_executable ]] || return 1
  argv0=$(tr '\0' '\n' </proc/$pid/cmdline 2>/dev/null | sed -n '1p') || return 1
  [[ -n $argv0 && ${argv0:t} == ${expected_executable:t} ]] || return 1
  taskset -pc $pid >/dev/null 2>&1 || return 1
}

verify_resolved_pid() {
  emulate -L zsh
  setopt TYPESET_SILENT
  local pid=$1
  local expected_executable=$2
  local actual_executable argv0

  if [[ $pid != <-> ]]; then
    print -u2 -- "PID is not one decimal integer: $pid"
    return 1
  fi
  if [[ ! -d /proc/$pid ]]; then
    print -u2 -- "PID has no live /proc entry: $pid"
    return 1
  fi
  actual_executable=$(readlink -f /proc/$pid/exe 2>/dev/null) || {
    print -u2 -- "cannot resolve /proc/$pid/exe"
    return 1
  }
  if [[ $actual_executable != $expected_executable ]]; then
    print -u2 -- "PID $pid executable mismatch: $actual_executable"
    return 1
  fi
  argv0=$(tr '\0' '\n' </proc/$pid/cmdline 2>/dev/null | sed -n '1p') || {
    print -u2 -- "cannot read /proc/$pid/cmdline"
    return 1
  }
  if [[ -z $argv0 || ${argv0:t} != ${expected_executable:t} ]]; then
    print -u2 -- "PID $pid command line does not identify ${expected_executable:t}: $argv0"
    return 1
  fi
  taskset -pc $pid >/dev/null 2>&1 || {
    print -u2 -- "cannot query affinity for PID $pid"
    return 1
  }
}

resolve_unique_child_executable() {
  emulate -L zsh
  setopt TYPESET_SILENT
  local parent_pid=$1
  local expected_executable=$2
  local max_attempts=${3:-500}
  local attempt candidate
  local -a candidates valid_candidates

  if [[ $parent_pid != <-> || $max_attempts != <-> || $max_attempts == 0 ]]; then
    print -u2 -- "invalid child resolver arguments"
    return 1
  fi

  for (( attempt = 1; attempt <= max_attempts; ++attempt )); do
    candidates=($(pgrep -P $parent_pid 2>/dev/null || true))
    valid_candidates=()
    for candidate in $candidates; do
      if _pid_matches_expected_process_quiet $candidate $expected_executable; then
        valid_candidates+=($candidate)
      fi
    done
    if (( ${#valid_candidates} == 1 )); then
      print -r -- $valid_candidates[1]
      return 0
    fi
    if (( ${#valid_candidates} > 1 )); then
      print -u2 -- "ambiguous child resolver: ${#valid_candidates} matching PIDs for parent $parent_pid"
      return 1
    fi
    kill -0 $parent_pid 2>/dev/null || {
      print -u2 -- "child resolver owner exited before a matching PID was found: $parent_pid"
      return 1
    }
    sleep 0.01
  done

  print -u2 -- "child resolver found zero matching PIDs for parent $parent_pid"
  return 1
}

resolve_unique_group_executable() {
  emulate -L zsh
  setopt TYPESET_SILENT
  local group_id=$1
  local expected_executable=$2
  local max_attempts=${3:-500}
  local attempt candidate
  local -a candidates valid_candidates

  if [[ $group_id != <-> || $max_attempts != <-> || $max_attempts == 0 ]]; then
    print -u2 -- "invalid process-group resolver arguments"
    return 1
  fi

  for (( attempt = 1; attempt <= max_attempts; ++attempt )); do
    candidates=($(ps -eo pid=,pgid= | awk -v group_id=$group_id '$2 == group_id {print $1}'))
    valid_candidates=()
    for candidate in $candidates; do
      if _pid_matches_expected_process_quiet $candidate $expected_executable; then
        valid_candidates+=($candidate)
      fi
    done
    if (( ${#valid_candidates} == 1 )); then
      print -r -- $valid_candidates[1]
      return 0
    fi
    if (( ${#valid_candidates} > 1 )); then
      print -u2 -- "ambiguous process-group resolver: ${#valid_candidates} matching PIDs in PGID $group_id"
      return 1
    fi
    kill -0 $group_id 2>/dev/null || {
      print -u2 -- "process-group resolver owner exited before a matching PID was found: $group_id"
      return 1
    }
    sleep 0.01
  done

  print -u2 -- "process-group resolver found zero matching PIDs in PGID $group_id"
  return 1
}
