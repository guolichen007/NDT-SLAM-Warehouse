#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <string>

#include <sys/stat.h>
#include <sys/types.h>

#include "ndt_slam/runtime_diagnostics.hpp"

namespace ndt_slam {
namespace {

void configureDiagnostics(RuntimeDiagnostics* diagnostics) {
  RuntimeDiagnosticsConfig config;
  config.enabled = true;
  config.csv_enabled = false;
  config.console_risk_enabled = true;
  config.risk_repeat_period_sec = 5.0;
  diagnostics->configure(config, ".");
}

TEST(RuntimeDiagnosticsTest, ConsoleRiskDisabledProducesNoRiskTerminalOutput) {
  RuntimeDiagnostics diagnostics;
  RuntimeDiagnosticsConfig config;
  config.enabled = true;
  config.csv_enabled = false;
  config.console_risk_enabled = false;
  diagnostics.configure(config, ".");

  testing::internal::CaptureStdout();
  diagnostics.logNdtRiskNotConverged(
      1, 1.0, 2.0, 20, "active", 3000, 3000, 40.0, 60.0);
  const std::string output = testing::internal::GetCapturedStdout();
  EXPECT_TRUE(output.empty());
}

TEST(RuntimeDiagnosticsTest, PipelineRiskIsEnterChangeRepeatSuppressedAndClear) {
  RuntimeDiagnostics diagnostics;
  configureDiagnostics(&diagnostics);
  PipelineRiskRecord record;
  record.reason = "FRAME_OVERRUN";
  record.level = 1;
  record.frame = 10;
  record.stamp = 12.5;
  record.frame_budget_ms = 100.0;
  record.total_ms = 130.0;
  record.consecutive_overruns = 1;

  testing::internal::CaptureStdout();
  diagnostics.updatePipelineRisk(record);
  diagnostics.updatePipelineRisk(record);
  const std::string entered = testing::internal::GetCapturedStdout();
  EXPECT_NE(entered.find("[PIPELINE_RISK_ENTER]"), std::string::npos);
  EXPECT_EQ(entered.find("[PIPELINE_RISK_REPEAT]"), std::string::npos);

  record.reason = "SUSTAINED_OVERRUN";
  record.level = 2;
  record.consecutive_overruns = 3;
  testing::internal::CaptureStdout();
  diagnostics.updatePipelineRisk(record);
  const std::string changed = testing::internal::GetCapturedStdout();
  EXPECT_NE(changed.find("[PIPELINE_RISK_CHANGE]"), std::string::npos);
  EXPECT_NE(changed.find("previous_reason=FRAME_OVERRUN"),
            std::string::npos);

  testing::internal::CaptureStdout();
  diagnostics.clearPipelineRisk(record);
  diagnostics.clearPipelineRisk(record);
  const std::string cleared = testing::internal::GetCapturedStdout();
  EXPECT_NE(cleared.find("[PIPELINE_RISK_CLEAR]"), std::string::npos);
}

TEST(RuntimeDiagnosticsTest, NonPipelineRiskEmitsOnceUntilRecovery) {
  RuntimeDiagnostics diagnostics;
  configureDiagnostics(&diagnostics);

  testing::internal::CaptureStdout();
  diagnostics.logNdtRiskNotConverged(
      1, 1.0, 2.0, 20, "active", 3000, 3000, 40.0, 60.0);
  diagnostics.logNdtRiskNotConverged(
      2, 1.1, 2.1, 20, "active", 3000, 3000, 41.0, 61.0);
  const std::string active = testing::internal::GetCapturedStdout();
  EXPECT_NE(active.find("[NDT_RISK] reason=NOT_CONVERGED"),
            std::string::npos);
  EXPECT_EQ(active.find("frame=2"), std::string::npos);

  testing::internal::CaptureStdout();
  diagnostics.clearConsoleRisk("NDT_NOT_CONVERGED", "NDT_RISK");
  diagnostics.clearConsoleRisk("NDT_NOT_CONVERGED", "NDT_RISK");
  const std::string recovered = testing::internal::GetCapturedStdout();
  EXPECT_NE(recovered.find("[NDT_RISK_CLEAR]"), std::string::npos);

  testing::internal::CaptureStdout();
  diagnostics.logNdtRiskNotConverged(
      3, 1.2, 2.2, 20, "active", 3000, 3000, 42.0, 62.0);
  const std::string reentered = testing::internal::GetCapturedStdout();
  EXPECT_NE(reentered.find("frame=3"), std::string::npos);
}

TEST(RuntimeDiagnosticsTest, CargoCsvSchemaColumnCountsMatch) {
  const std::string dir = "/tmp/ndt_slam_csv_schema_test";
  ::mkdir(dir.c_str(), 0755);
  const std::string cargo_path = dir + "/cargo_frames.csv";
  const std::string static_path = dir + "/static_evidence.csv";
  const std::string ndt_path = dir + "/runtime_frames.csv";
  std::remove(cargo_path.c_str());
  std::remove(static_path.c_str());
  std::remove(ndt_path.c_str());

  RuntimeDiagnostics diagnostics;
  RuntimeDiagnosticsConfig config;
  config.enabled = true;
  config.csv_enabled = true;
  diagnostics.configure(config, dir);

  CargoFrameRecord rec;
  rec.stamp = 1.0;
  rec.raw_safety_reason = "vertical_continuity_insufficient_low_clearance";
  rec.raw_cluster_present = true;
  rec.raw_cluster_distance_m = 2.0;
  rec.raw_cluster_top_z95_m = 1.2;
  rec.raw_cluster_bottom_z05_m = 0.5;
  rec.raw_cluster_vertical_span_m = 0.7;
  rec.raw_cluster_vertical_continuity_ratio = 0.3;
  rec.raw_cluster_vertical_continuity_threshold = 0.45;
  rec.raw_cluster_conservative_clearance_m = 0.55;
  rec.candidate_top1_rank = 1.0;
  rec.candidate_top2_rank = 2.0;
  rec.candidate_rank_margin = 0.3;
  diagnostics.writeCargoFrame(rec);
  diagnostics.flushCsv();

  std::ifstream in(cargo_path);
  ASSERT_TRUE(in.is_open());
  std::string header;
  std::string row;
  ASSERT_TRUE(std::getline(in, header));
  ASSERT_TRUE(std::getline(in, row));

  const auto columns = [](const std::string& line) {
    return std::count(line.begin(), line.end(), ',') + 1;
  };
  EXPECT_EQ(columns(header), columns(row));
  EXPECT_GT(columns(header), 0);

  std::remove(cargo_path.c_str());
  std::remove(static_path.c_str());
  std::remove(ndt_path.c_str());
}

}  // namespace
}  // namespace ndt_slam
