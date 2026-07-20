#include <gtest/gtest.h>

#include <string>

#include "ndt_slam/runtime_diagnostics.hpp"

namespace ndt_slam {
namespace {

void configureDiagnostics(RuntimeDiagnostics* diagnostics) {
  RuntimeDiagnosticsConfig config;
  config.enabled = true;
  config.csv_enabled = false;
  config.console_period_sec = 5.0;
  config.risk_repeat_period_sec = 5.0;
  diagnostics->configure(config, ".");
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

}  // namespace
}  // namespace ndt_slam
