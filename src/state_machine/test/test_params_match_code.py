#!/usr/bin/env python3
# Copyright 2026 2026_IFAC contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""state_machine.yaml의 키와 노드가 선언한 파라미터가 정확히 일치하는지 검사한다.

왜 필요한가 (2026-08-16): ROS 2는 params 파일의 **미선언 키를 조용히 무시**한다. 그래서
둘이 어긋나도 노드는 정상 기동하고 아무 경고도 없다. 두 방향 모두 위험하다:

  yaml에만 있는 키   → 설정한 줄 알았는데 아무 효과가 없다(죽은 설정).
  코드에만 있는 키   → 코드 기본값이 조용히 쓰인다. 의도가 어디에도 안 적혀 있다.

실해가 두 번 있었다.
  1. 2026-08-16 오전: enter_global_tail_distance_m가 플래너 6.0 / FSM 1.0으로 어긋나
     8랩 동안 AVOID→GLOBAL 전이가 0회였다. 아무 에러도 나지 않았다.
  2. 2026-08-16 오후 팀 병합: 이 yaml이 옛 버전으로 되돌아가면서 계약 키 3개
     (handoff_ot_line, enter_global_tail_distance_m, avoid_path_liveness_timeout_sec)가
     사라지고, 이미 코드에서 제거된 stopped_path_* 계열 5개와 옛 비율 키가 되살아났다.
     죽은 키 10개 / 누락 3개였는데 노드는 멀쩡히 떴다.
"""
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
SOURCE = HERE.parent / 'src' / 'state_machine_node.cpp'
YAML = HERE.parent / 'config' / 'state_machine.yaml'


def declared_in_code(text):
    return set(re.findall(r'declare_parameter<[^>]*>\(\s*"([a-z_0-9]+)"', text))


def keys_in_yaml(text):
    # ros__parameters 블록의 4-space 들여쓰기 키만. 주석과 리스트 원소는 제외된다.
    return set(re.findall(r'^    ([a-z_0-9]+):', text, re.M))


def main():
    code = declared_in_code(SOURCE.read_text())
    config = keys_in_yaml(YAML.read_text())
    if not code:
        print(f'FAIL: {SOURCE}에서 declare_parameter를 하나도 찾지 못했다')
        return 1
    dead = sorted(config - code)
    missing = sorted(code - config)
    ok = True
    if dead:
        print('FAIL: yaml에만 있는 키 (코드가 읽지 않음 — 설정해도 효과 없음):')
        for name in dead:
            print(f'  - {name}')
        ok = False
    if missing:
        print('FAIL: 코드에만 있는 키 (yaml에 없어 기본값이 조용히 쓰임):')
        for name in missing:
            print(f'  - {name}')
        ok = False
    if ok:
        print(f'OK: 파라미터 {len(code)}개가 코드와 yaml에서 일치합니다.')
        return 0
    return 1


def test_state_machine_parameters_match_code():
    """yaml과 코드가 어긋나도 노드는 조용히 뜨므로, 검사는 여기서 해야 한다."""
    assert main() == 0


if __name__ == '__main__':
    sys.exit(main())
