// Copyright 2026 2026_IFAC contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "local_planning/research_instrumentation.hpp"

namespace local_planning
{
namespace
{

std::string readAll(const std::filesystem::path & path)
{
  std::ifstream input(path);
  return std::string(
    std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

TEST(ResearchInstrumentation, WritesVersionedPlanningAndCandidateEventsAndFlushes)
{
  const auto unique = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = std::filesystem::temp_directory_path() /
    ("local_planning_research_test_" + std::to_string(unique));
  PlanningResearchConfig config;
  config.output_root = root.string();
  config.run_id = "run";
  config.scenario_id = "scenario";
  config.git_commit = "deadbeef";
  config.git_dirty_status = "clean";
  config.source_sha256 = "source";
  config.config_sha256 = "config";
  config.effective_parameter_snapshot_id = "parameters";
  config.effective_parameters_json = "{\"p3_mode\":\"TEST_ACTIVE\"}";
  config.queue_capacity = 2U;

  {
    PlanningResearchLogger logger(config);
    PlanningResearchCycle cycle;
    cycle.run_id = config.run_id;
    cycle.scenario_id = config.scenario_id;
    cycle.callback_sequence = 7U;
    cycle.ego_pose_source = "TEST";
    P3ResearchEvaluationRecord evaluation;
    evaluation.lineage.evaluation_sequence = 3U;
    evaluation.lineage.evaluation_role = "TEST_ROLE";
    evaluation.lineage.input_snapshot_id = "input_test";
    evaluation.lineage.ego_snapshot_id = "ego_test";
    evaluation.lineage.obstacle_snapshot_id = "obs_test";
    evaluation.lineage.reference_snapshot_id = "ref_test";
    evaluation.lineage.source_stamp_ns = 1234;
    evaluation.lineage.obstacle_sequence = 9U;
    evaluation.lineage.source_epoch = 2U;
    evaluation.lineage.reference_generation = 5U;
    evaluation.r3_invoked = true;
    evaluation.r3_method_name = "R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12";
    evaluation.r3_method_sha256 =
      "7b861e8c7e23ae168413885ea0dc2d09769046ff48a562700b658e084d116fcc";
    evaluation.r3_lateral_factor_count = 42U;
    evaluation.r3_pair_priority_count = 84U;
    evaluation.r3_lexicographic_factor_count = 10U;
    evaluation.r3_coverage_factor_count = 2U;
    evaluation.r3_constructed_candidate_count = 12U;
    evaluation.r3_path_digest_duplicate_count = 1U;
    evaluation.r3_validator_call_count = 11U;
    evaluation.r3_hard_valid_count = 3U;
    evaluation.r3_usable_valid_count = 2U;
    evaluation.r3_fallback_after_failure = "NONE";
    P3R3SelectedFactorTrace selected_factor;
    selected_factor.rank_stream = "LEXICOGRAPHIC";
    selected_factor.stream_rank = 4U;
    selected_factor.path_digest = "r3_digest";
    selected_factor.validator_executed = true;
    selected_factor.hard_valid = true;
    selected_factor.usable_valid = true;
    evaluation.r3_selected_factors.push_back(selected_factor);
    cycle.r3_invocation_count = 1U;
    cycle.r3_constructed_candidate_count = 12U;
    cycle.r3_path_digest_duplicate_count = 1U;
    cycle.r3_validator_call_count = 11U;
    cycle.r3_hard_valid_count = 3U;
    cycle.r3_usable_valid_count = 2U;
    P3ResearchCandidateRecord candidate;
    candidate.generator_stage = "M1";
    candidate.candidate_identity = "M1_TEST";
    candidate.s_probe = 4.0;
    candidate.d_probe = -0.25;
    candidate.root_index = 1;
    candidate.r3_rank_stream = "LEXICOGRAPHIC";
    candidate.r3_target_source = "PRODUCTION_TARGET";
    candidate.r3_mid_source = "TARGET_PLUS_0.05M";
    candidate.usable_valid = true;
    candidate.waypoint0_footprint_track_margin_m = 0.125;
    evaluation.candidates.push_back(candidate);
    cycle.evaluations.push_back(evaluation);
    EXPECT_TRUE(logger.submit(std::move(cycle)));
    EXPECT_EQ(logger.droppedCount(), 0U);
  }

  const auto run = root / "run";
  const std::string metadata = readAll(run / "metadata.json");
  const std::string planning = readAll(run / "planning_events.jsonl");
  const std::string evaluations = readAll(run / "evaluation_events.jsonl");
  const std::string candidates = readAll(run / "candidate_events.jsonl");
  const std::string summary = readAll(run / "run_summary.json");
  EXPECT_NE(metadata.find(kPlanningResearchSchemaVersion), std::string::npos);
  EXPECT_NE(metadata.find("effective_parameters"), std::string::npos);
  EXPECT_NE(planning.find("\"callback_sequence\":7"), std::string::npos);
  EXPECT_NE(planning.find("\"git_commit\":\"deadbeef\""), std::string::npos);
  EXPECT_NE(planning.find("\"effective_parameter_snapshot_id\":\"parameters\""),
    std::string::npos);
  EXPECT_NE(planning.find("\"dropped_log_count\":0"), std::string::npos);
  EXPECT_NE(planning.find("\"r3_invocation_count\":1"), std::string::npos);
  EXPECT_NE(planning.find("\"r3_validator_call_count\":11"), std::string::npos);
  EXPECT_NE(evaluations.find("\"event_type\":\"EVALUATION_EVENT\""), std::string::npos);
  EXPECT_NE(evaluations.find("\"evaluation_sequence\":3"), std::string::npos);
  EXPECT_NE(evaluations.find("\"evaluation_role\":\"TEST_ROLE\""), std::string::npos);
  EXPECT_NE(evaluations.find("\"input_snapshot_id\":\"input_test\""), std::string::npos);
  EXPECT_NE(evaluations.find("\"source_stamp_ns\":1234"), std::string::npos);
  EXPECT_NE(evaluations.find("\"method_name\":\"R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12\""),
    std::string::npos);
  EXPECT_NE(evaluations.find(
        "\"method_sha256\":\"7b861e8c7e23ae168413885ea0dc2d09769046ff48a562700b658e084d116fcc\""),
    std::string::npos);
  EXPECT_NE(evaluations.find("\"constructed_candidate_count\":12"), std::string::npos);
  EXPECT_NE(evaluations.find("\"validator_call_count\":11"), std::string::npos);
  EXPECT_NE(evaluations.find("\"rank_stream\":\"LEXICOGRAPHIC\""), std::string::npos);
  EXPECT_NE(candidates.find("\"generator_stage\":\"M1\""), std::string::npos);
  EXPECT_NE(candidates.find("\"evaluation_sequence\":3"), std::string::npos);
  EXPECT_NE(candidates.find("\"reference_snapshot_id\":\"ref_test\""), std::string::npos);
  EXPECT_NE(candidates.find("\"s_probe\":4"), std::string::npos);
  EXPECT_NE(candidates.find("\"d_probe\":-0.25"), std::string::npos);
  EXPECT_NE(candidates.find("\"root_index\":1"), std::string::npos);
  EXPECT_NE(candidates.find("\"usable_valid\":true"), std::string::npos);
  EXPECT_NE(candidates.find("\"target_source\":\"PRODUCTION_TARGET\""), std::string::npos);
  EXPECT_NE(candidates.find("\"footprint_track_margin_m\":0.125"), std::string::npos);
  EXPECT_NE(summary.find("\"clean_shutdown\":true"), std::string::npos);
  EXPECT_NE(summary.find("\"written_cycle_count\":1"), std::string::npos);
  EXPECT_NE(summary.find("\"dropped_log_count\":0"), std::string::npos);
  EXPECT_THROW(PlanningResearchLogger duplicate(config), std::runtime_error);
}

TEST(ResearchInstrumentation, RejectsMissingProvenanceBeforeCreatingRun)
{
  PlanningResearchConfig config;
  config.output_root = std::filesystem::temp_directory_path().string();
  config.run_id = "missing_provenance";
  EXPECT_THROW(PlanningResearchLogger logger(config), std::invalid_argument);
}

}  // namespace
}  // namespace local_planning
