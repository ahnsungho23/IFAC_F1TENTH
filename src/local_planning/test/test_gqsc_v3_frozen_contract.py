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
"""Pin the canonical GQSC-v3 bytes and the production compile-time contract together."""

from __future__ import annotations

import hashlib
from pathlib import Path
import re


PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
METHOD = REPO / 'planning_study/gqsc_v3_geometry_normalized/gqsc_v3_geometry_general_method.json'
HEADER = PACKAGE / 'include/local_planning/gqsc_v3_frozen_contract.hpp'
EXPECTED_METHOD_SHA = '965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780'
EXPECTED_LOCK_SHA = 'ea30a441695732e87d43d6863e1a31992dbfdd45749d324047495b6c96824185'


def test_canonical_method_bytes_match_production_contract() -> None:
    observed = hashlib.sha256(METHOD.read_bytes()).hexdigest()
    assert observed == EXPECTED_METHOD_SHA
    text = HEADER.read_text(encoding='utf-8')
    assert EXPECTED_METHOD_SHA in text
    assert EXPECTED_LOCK_SHA in text
    assert re.search(r'kGqscV3PairProxyBudget\s*=\s*128U', text)
    assert re.search(r'kGqscV3ReconstructionBudget\s*=\s*12U', text)
    assert re.search(r'kGqscV3ValidatorBudget\s*=\s*12U', text)
    assert 'V3_SIDE_BALANCED_DISJOINT' in text
    assert 'add_component_half_far002_inward015 = false' in text
