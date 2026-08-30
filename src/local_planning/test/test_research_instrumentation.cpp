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
    P3ResearchCandidateRecord candidate;
    candidate.generator_stage = "M1";
    candidate.candidate_identity = "M1_TEST";
    candidate.s_probe = 4.0;
    candidate.d_probe = -0.25;
    candidate.root_index = 1;
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
  EXPECT_NE(evaluations.find("\"event_type\":\"EVALUATION_EVENT\""), std::string::npos);
  EXPECT_NE(evaluations.find("\"evaluation_sequence\":3"), std::string::npos);
  EXPECT_NE(evaluations.find("\"evaluation_role\":\"TEST_ROLE\""), std::string::npos);
  EXPECT_NE(evaluations.find("\"input_snapshot_id\":\"input_test\""), std::string::npos);
  EXPECT_NE(evaluations.find("\"source_stamp_ns\":1234"), std::string::npos);
  EXPECT_NE(candidates.find("\"generator_stage\":\"M1\""), std::string::npos);
  EXPECT_NE(candidates.find("\"evaluation_sequence\":3"), std::string::npos);
  EXPECT_NE(candidates.find("\"reference_snapshot_id\":\"ref_test\""), std::string::npos);
  EXPECT_NE(candidates.find("\"s_probe\":4"), std::string::npos);
  EXPECT_NE(candidates.find("\"d_probe\":-0.25"), std::string::npos);
  EXPECT_NE(candidates.find("\"root_index\":1"), std::string::npos);
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
