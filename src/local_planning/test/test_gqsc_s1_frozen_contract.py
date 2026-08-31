#!/usr/bin/env python3
# Copyright 2026 2026_IFAC contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Pin S1 bytes, immutable v3 composition, and the production compile-time contract."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re


PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
METHOD = REPO / 'planning_study/gqsc_s1_closed_loop_candidate_v1/gqsc_s1_method.json'
BASE = REPO / 'planning_study/gqsc_v3_geometry_normalized/gqsc_v3_geometry_general_method.json'
HEADER = PACKAGE / 'include/local_planning/gqsc_s1_frozen_contract.hpp'
EXPECTED_METHOD_SHA = '670f39a23479bcdcc1db0829a895257443fec8ee2f78f186bc1beb224090b776'
EXPECTED_BASE_SHA = '965f6ce65b7ce5b1c6426a0975c1dfafe779e89eb20308bb09a11a1b4a22e780'


def test_canonical_s1_composition_matches_production_contract() -> None:
    assert hashlib.sha256(METHOD.read_bytes()).hexdigest() == EXPECTED_METHOD_SHA
    assert hashlib.sha256(BASE.read_bytes()).hexdigest() == EXPECTED_BASE_SHA
    method = json.loads(METHOD.read_text(encoding='utf-8'))
    assert method['base_method']['sha256'] == EXPECTED_BASE_SHA
    assert method['only_method_override']['selection_policy'] == (
        'LEX8_GLOBAL_DISJOINT_COVERAGE4')
    text = HEADER.read_text(encoding='utf-8')
    assert EXPECTED_METHOD_SHA in text
    assert EXPECTED_BASE_SHA in text
    assert re.search(r'kGqscS1PairProxyBudget\s*=\s*128U', text)
    assert re.search(r'kGqscS1LexicographicQuota\s*=\s*8U', text)
    assert re.search(r'kGqscS1CoverageQuota\s*=\s*4U', text)
    assert re.search(r'kGqscS1ReconstructionBudget\s*=\s*12U', text)
    assert re.search(r'kGqscS1ValidatorBudget\s*=\s*12U', text)
    assert 'P3R3RTStandaloneDiversityPolicy::DISJOINT_COVERAGE' in text
    assert 'add_component_half_far002_inward015 = false' in text
