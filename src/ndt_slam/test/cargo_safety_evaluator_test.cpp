#include "ndt_slam/cargo_safety_evaluator.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace ndt_slam {
namespace {

CargoSafetyInput baseInput() {
    CargoSafetyInput input;
    input.height.valid = true;
    input.height.stale = false;
    input.height.bottom_z = 2.0F;
    input.height.bottom_uncertainty_m = 0.10F;
    input.height.stamp_sec = 10.0;
    input.evaluation_time_sec = 10.0;
    input.footprint_base.valid = true;
    input.footprint_base.center_base.setZero();
    input.footprint_base.length_m = 1.0F;
    input.footprint_base.width_m = 1.0F;
    input.footprint_base.yaw_base_rad = 0.0F;
    input.footprint_base.min_z = 1.9F;
    input.footprint_base.max_z = 3.0F;
    input.obstacle_cloud_base.reset(new pcl::PointCloud<pcl::PointXYZ>);
    input.obstacle_observation_valid = true;
    input.obstacle_cloud_age_sec = 0.0;
    input.obstacle_roi_finite_points = 100;
    input.obstacle_roi_coverage_ratio = 0.80F;
    return input;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr mutableObstacleCloud(
        CargoSafetyInput* input) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    if (input != nullptr && input->obstacle_cloud_base) {
        *cloud = *input->obstacle_cloud_base;
    }

    if (input != nullptr) {
        input->obstacle_cloud_base = cloud;
    }

    return cloud;
}

void addCluster(CargoSafetyInput* input,
                float footprint_distance,
                float top_z,
                float y = 0.0F) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (input->obstacle_cloud_base) *cloud = *input->obstacle_cloud_base;
    const float x = 0.5F + footprint_distance;
    for (int i = 0; i < 8; ++i) {
        cloud->push_back(pcl::PointXYZ(
            x + 0.005F * static_cast<float>(i),
            y + 0.004F * static_cast<float>(i % 3),
            top_z - 0.01F * static_cast<float>(i % 2)));
    }
    input->obstacle_cloud_base = cloud;
}

TEST(CargoSafetyConfig, ProductionDefaultsMatchContract) {
    const CargoSafetyConfig config;
    EXPECT_FLOAT_EQ(config.level1_distance_m, 3.0F);
    EXPECT_FLOAT_EQ(config.level2_distance_m, 5.0F);
    EXPECT_FLOAT_EQ(config.minimum_vertical_clearance_m, 0.80F);
    EXPECT_FLOAT_EQ(config.minimum_roi_coverage_ratio, 0.05F);
    EXPECT_DOUBLE_EQ(config.maximum_obstacle_cloud_age_sec, 0.50);
}

CargoSafetyDecisionInput baseDecision(
    HookLoadSignalRole role = HookLoadSignalRole::AUXILIARY) {
    CargoSafetyDecisionInput input;
    input.system_ready = true;
    input.localization_valid = true;
    input.hook_signal_role = role;
    return input;
}

CargoSafetyDecisionInput strictEmptyDecision(HookLoadSignalRole role) {
    CargoSafetyDecisionInput input = baseDecision(role);
    input.safe_empty = true;
    if (role == HookLoadSignalRole::REQUIRED) {
        input.gravity_valid = true;
        input.gravity_empty = true;
    }
    return input;
}

CargoSafetyDecisionInput formalDecision(std::uint16_t warning_code) {
    CargoSafetyDecisionInput input = baseDecision();
    input.warning_valid = true;
    input.formal_cargo_valid = true;
    input.formal_clear_authorized = true;
    input.obstacle_evidence_ready = true;
    input.warning_code = warning_code;
    input.evidence_reason = "test_evidence";
    return input;
}

CargoSafetyDecisionInput pendingDecision(std::uint16_t warning_code) {
    CargoSafetyDecisionInput input = baseDecision();
    input.warning_valid = true;
    input.pending_positive_warning = true;
    input.obstacle_evidence_ready = true;
    input.warning_code = warning_code;
    return input;
}

TEST(CargoSafetyDecision, EndToEndStatusCodePriorityAndFaultMask) {
    EXPECT_EQ(composeCargoSafetyDecision(strictEmptyDecision(
                  HookLoadSignalRole::REQUIRED)).requested_code,
              CargoSafetyProtocol::kClear);
    for (std::uint16_t warning : {
             static_cast<std::uint16_t>(CargoSafetyProtocol::kClear),
             static_cast<std::uint16_t>(CargoSafetyProtocol::kLevel1Warning),
             static_cast<std::uint16_t>(CargoSafetyProtocol::kLevel2Warning),
             static_cast<std::uint16_t>(CargoSafetyProtocol::kAnomalyReview)}) {
        const CargoSafetyDecision result =
            composeCargoSafetyDecision(formalDecision(warning));
        EXPECT_TRUE(result.valid);
        EXPECT_EQ(result.requested_code, warning);
        EXPECT_EQ(result.fault_code, 0);
    }

    CargoSafetyDecisionInput multiple = formalDecision(
        CargoSafetyProtocol::kLevel1Warning);
    multiple.localization_valid = false;
    multiple.gravity_valid = false;
    const CargoSafetyDecision localization =
        composeCargoSafetyDecision(multiple);
    EXPECT_EQ(localization.requested_code,
              CargoSafetyProtocol::kLocalizationInvalid);
    EXPECT_NE(localization.fault_mask &
                  CargoSafetyProtocol::kFaultLocalization, 0U);
    CargoSafetyDecisionInput cargo = baseDecision();
    cargo.cargo_fault = true;
    EXPECT_EQ(composeCargoSafetyDecision(cargo).requested_code,
              CargoSafetyProtocol::kCargoInvalid);

    CargoSafetyDecisionInput obstacle = formalDecision(
        CargoSafetyProtocol::kClear);
    obstacle.obstacle_fault = true;
    EXPECT_EQ(composeCargoSafetyDecision(obstacle).requested_code,
              CargoSafetyProtocol::kObstacleInvalid);

    CargoSafetyDecisionInput internal = formalDecision(
        CargoSafetyProtocol::kLevel2Warning);
    internal.internal_fault = true;
    EXPECT_EQ(composeCargoSafetyDecision(internal).requested_code,
              CargoSafetyProtocol::kInternalError);
}

TEST(CargoSafetyDecision, SystemAndGravityFaultsNeverBecomeLevel2Warning) {
    CargoSafetyDecisionInput startup = formalDecision(
        CargoSafetyProtocol::kLevel2Warning);
    startup.system_ready = false;
    startup.evidence_reason = "cargo_track_not_initialized";
    const CargoSafetyDecision startup_result =
        composeCargoSafetyDecision(startup);
    EXPECT_EQ(startup_result.requested_code,
              CargoSafetyProtocol::kSystemNotReady);
    EXPECT_EQ(startup_result.reason, "cargo_track_not_initialized");

    CargoSafetyDecisionInput gravity = formalDecision(
        CargoSafetyProtocol::kLevel2Warning);
    gravity.hook_signal_role = HookLoadSignalRole::REQUIRED;
    gravity.gravity_valid = false;
    EXPECT_EQ(composeCargoSafetyDecision(gravity).requested_code,
              CargoSafetyProtocol::kGravityInvalid);
}

TEST(CargoSafetyDecision, AuxiliaryMissingGravityDoesNotCreate32) {
    const auto result = composeCargoSafetyDecision(
        strictEmptyDecision(HookLoadSignalRole::AUXILIARY));
    EXPECT_EQ(result.requested_code, CargoSafetyProtocol::kClear);
    EXPECT_EQ(result.fault_mask & CargoSafetyProtocol::kFaultGravity, 0U);
}

TEST(CargoSafetyDecision, DisabledGravityDoesNotCreate32) {
    auto input = strictEmptyDecision(HookLoadSignalRole::DISABLED);
    input.gravity_valid = true;
    input.gravity_loaded = true;
    input.gravity_conflict = true;
    const auto result = composeCargoSafetyDecision(input);
    EXPECT_EQ(result.requested_code, CargoSafetyProtocol::kClear);
    EXPECT_EQ(result.fault_mask & CargoSafetyProtocol::kFaultGravity, 0U);
}

TEST(CargoSafetyDecision, AuxiliaryFreshLoadedAgainstLidarEmptyCannotClear) {
    auto input = strictEmptyDecision(HookLoadSignalRole::AUXILIARY);
    input.gravity_valid = true;
    input.gravity_loaded = true;
    input.hook_loaded = true;
    input.gravity_conflict = true;
    EXPECT_EQ(composeCargoSafetyDecision(input).requested_code,
              CargoSafetyProtocol::kCargoInvalid);
}

TEST(CargoSafetyDecision, AuxiliaryFreshEmptyAgainstFormalCargoCannotClear) {
    auto input = formalDecision(CargoSafetyProtocol::kClear);
    input.gravity_valid = true;
    input.gravity_empty = true;
    input.gravity_conflict = true;
    EXPECT_EQ(composeCargoSafetyDecision(input).requested_code,
              CargoSafetyProtocol::kCargoInvalid);
}

TEST(CargoSafetyDecision, RequiredFreshEmptyCanClearOnlyWithLidarEmpty) {
    const auto clear = composeCargoSafetyDecision(
        strictEmptyDecision(HookLoadSignalRole::REQUIRED));
    EXPECT_EQ(clear.requested_code, CargoSafetyProtocol::kClear);
    auto no_lidar_empty = baseDecision(HookLoadSignalRole::REQUIRED);
    no_lidar_empty.gravity_valid = true;
    no_lidar_empty.gravity_empty = true;
    EXPECT_EQ(composeCargoSafetyDecision(no_lidar_empty).requested_code,
              CargoSafetyProtocol::kCargoInvalid);
}

TEST(CargoSafetyDecision, SafeEmptyHardContractRejectsContradictions) {
    auto hook = strictEmptyDecision(HookLoadSignalRole::AUXILIARY);
    hook.hook_loaded = true;
    EXPECT_EQ(composeCargoSafetyDecision(hook).requested_code,
              CargoSafetyProtocol::kInternalError);
    auto formal = strictEmptyDecision(HookLoadSignalRole::AUXILIARY);
    formal.formal_cargo_valid = true;
    EXPECT_EQ(composeCargoSafetyDecision(formal).requested_code,
              CargoSafetyProtocol::kInternalError);
    auto cargo_fault = strictEmptyDecision(HookLoadSignalRole::AUXILIARY);
    cargo_fault.cargo_fault = true;
    EXPECT_EQ(composeCargoSafetyDecision(cargo_fault).requested_code,
              CargoSafetyProtocol::kInternalError);
    auto obstacle_fault = strictEmptyDecision(HookLoadSignalRole::AUXILIARY);
    obstacle_fault.obstacle_fault = true;
    EXPECT_EQ(composeCargoSafetyDecision(obstacle_fault).requested_code,
              CargoSafetyProtocol::kInternalError);
}

TEST(CargoSafetyDecision, ClearDecisionHasZeroFaultMask) {
    const auto empty = composeCargoSafetyDecision(
        strictEmptyDecision(HookLoadSignalRole::REQUIRED));
    EXPECT_TRUE(cargoSafetyDecisionSelfConsistent(empty));
    EXPECT_EQ(empty.fault_mask, 0U);
    const auto formal = composeCargoSafetyDecision(
        formalDecision(CargoSafetyProtocol::kClear));
    EXPECT_TRUE(cargoSafetyDecisionSelfConsistent(formal));
    EXPECT_EQ(formal.fault_mask, 0U);
}

TEST(CargoSafetyDecision, ConflictPositiveHazardsAreRetained) {
    for (std::uint16_t code : {
             static_cast<std::uint16_t>(CargoSafetyProtocol::kLevel1Warning),
             static_cast<std::uint16_t>(CargoSafetyProtocol::kLevel2Warning)}) {
        auto input = formalDecision(code);
        input.gravity_valid = true;
        input.gravity_empty = true;
        input.gravity_conflict = true;
        const auto result = composeCargoSafetyDecision(input);
        EXPECT_EQ(result.requested_code, code);
        EXPECT_EQ(result.reason,
                  "gravity_lidar_conflict_hazard_retained");
    }
}

TEST(CargoSafetyDecision, PendingContract) {
    EXPECT_EQ(composeCargoSafetyDecision(baseDecision()).requested_code,
              CargoSafetyProtocol::kCargoInvalid);
    EXPECT_EQ(composeCargoSafetyDecision(pendingDecision(
                  CargoSafetyProtocol::kLevel1Warning)).requested_code,
              CargoSafetyProtocol::kLevel1Warning);
    EXPECT_EQ(composeCargoSafetyDecision(pendingDecision(
                  CargoSafetyProtocol::kLevel2Warning)).requested_code,
              CargoSafetyProtocol::kLevel2Warning);
    EXPECT_EQ(composeCargoSafetyDecision(pendingDecision(
                  CargoSafetyProtocol::kClear)).requested_code,
              CargoSafetyProtocol::kCargoInvalid);
}

TEST(CargoSafetyDecision, FormalClearRequiresAllAuthorities) {
    auto missing_live = formalDecision(CargoSafetyProtocol::kClear);
    missing_live.obstacle_evidence_ready = false;
    EXPECT_EQ(composeCargoSafetyDecision(missing_live).requested_code,
              CargoSafetyProtocol::kObstacleInvalid);
    auto missing_static = formalDecision(CargoSafetyProtocol::kClear);
    missing_static.formal_clear_authorized = false;
    EXPECT_EQ(composeCargoSafetyDecision(missing_static).requested_code,
              CargoSafetyProtocol::kObstacleInvalid);
    EXPECT_EQ(composeCargoSafetyDecision(formalDecision(
                  CargoSafetyProtocol::kClear)).requested_code,
              CargoSafetyProtocol::kClear);
}

TEST(CargoSafetyDecision,
     MissingCertifiedReferenceBlocksClearButRetainsPositiveHazard) {
    auto empty = strictEmptyDecision(HookLoadSignalRole::REQUIRED);
    empty.clear_authority_incomplete = true;
    EXPECT_EQ(composeCargoSafetyDecision(empty).requested_code,
              CargoSafetyProtocol::kObstacleInvalid);

    auto formal_clear = formalDecision(CargoSafetyProtocol::kClear);
    formal_clear.clear_authority_incomplete = true;
    EXPECT_EQ(composeCargoSafetyDecision(formal_clear).requested_code,
              CargoSafetyProtocol::kObstacleInvalid);

    auto positive = formalDecision(CargoSafetyProtocol::kLevel1Warning);
    positive.clear_authority_incomplete = true;
    EXPECT_EQ(composeCargoSafetyDecision(positive).requested_code,
              CargoSafetyProtocol::kLevel1Warning);
}

TEST(CargoSafetyDecision, NormalLifecycleNeverProduces35) {
    std::vector<CargoSafetyDecisionInput> lifecycle;
    CargoSafetyDecisionInput startup;
    lifecycle.push_back(startup);
    auto localization = baseDecision();
    localization.localization_valid = false;
    lifecycle.push_back(localization);
    lifecycle.push_back(baseDecision(HookLoadSignalRole::REQUIRED));
    lifecycle.push_back(baseDecision());  // candidate / clear-wait / pending
    auto obstacle_pending = formalDecision(CargoSafetyProtocol::kClear);
    obstacle_pending.obstacle_evidence_ready = false;
    lifecycle.push_back(obstacle_pending);  // locked / lost-hold
    lifecycle.push_back(pendingDecision(CargoSafetyProtocol::kLevel1Warning));
    lifecycle.push_back(pendingDecision(CargoSafetyProtocol::kLevel2Warning));
    lifecycle.push_back(formalDecision(CargoSafetyProtocol::kLevel1Warning));
    lifecycle.push_back(formalDecision(CargoSafetyProtocol::kLevel2Warning));
    lifecycle.push_back(formalDecision(CargoSafetyProtocol::kClear));
    lifecycle.push_back(strictEmptyDecision(HookLoadSignalRole::AUXILIARY));
    for (const auto& input : lifecycle) {
        EXPECT_NE(composeCargoSafetyDecision(input).requested_code,
                  CargoSafetyProtocol::kInternalError);
    }
}

TEST(CargoSafetyDecision, ExplicitImpossibleCombinationProduces35) {
    auto input = baseDecision();
    input.gravity_empty = true;
    input.gravity_loaded = true;
    const auto result = composeCargoSafetyDecision(input);
    EXPECT_EQ(result.requested_code, CargoSafetyProtocol::kInternalError);
    EXPECT_EQ(result.reason, "gravity_state_not_exclusive");
}

TEST(CargoSafetyEvaluator, ExactDistanceAndClearanceBoundaries) {
    CargoSafetyEvaluator evaluator;
    struct Case {
        float distance;
        float obstacle_top;
        std::uint16_t expected;
    };
    const Case cases[] = {
        {2.99F, 1.01F, CargoSafetyEvaluator::kLevel1Code},
        {3.00F, 1.01F, CargoSafetyEvaluator::kLevel1Code},
        {3.01F, 1.01F, CargoSafetyEvaluator::kLevel2Code},
        {5.00F, 1.01F, CargoSafetyEvaluator::kLevel2Code},
        {5.01F, 1.01F, CargoSafetyEvaluator::kSafeCode},
        {2.00F, 1.00F, CargoSafetyEvaluator::kSafeCode},
        {2.00F, 0.99F, CargoSafetyEvaluator::kSafeCode},
        {4.00F, 1.30F, CargoSafetyEvaluator::kLevel2Code},
    };
    for (const auto& test_case : cases) {
        CargoSafetyInput input = baseInput();
        addCluster(&input, test_case.distance, test_case.obstacle_top);
        const CargoSafetyResult result = evaluator.evaluate(input);
        ASSERT_TRUE(result.input_valid) << result.reason;
        ASSERT_TRUE(result.warning_valid) << result.reason;
        EXPECT_EQ(result.fault, CargoSafetyFault::NONE);
        EXPECT_EQ(result.warning_code, test_case.expected)
            << test_case.distance << " " << test_case.obstacle_top;
    }
}

TEST(CargoSafetyEvaluator, InvalidHeightNeverBecomesLevel2Warning) {
    CargoSafetyEvaluator evaluator;
    CargoSafetyInput invalid = baseInput();
    invalid.height.valid = false;
    const CargoSafetyResult invalid_result = evaluator.evaluate(invalid);
    EXPECT_FALSE(invalid_result.warning_valid);
    EXPECT_EQ(invalid_result.warning_code, 0U);
    EXPECT_EQ(invalid_result.fault, CargoSafetyFault::CARGO_HEIGHT_INVALID);

    CargoSafetyInput stale = baseInput();
    stale.evaluation_time_sec = 11.0;
    stale.height.stale = true;
    const CargoSafetyResult stale_result = evaluator.evaluate(stale);
    EXPECT_TRUE(stale_result.height_stale);
    EXPECT_FALSE(stale_result.warning_valid);
    EXPECT_EQ(stale_result.warning_code, 0U);
    EXPECT_EQ(stale_result.fault, CargoSafetyFault::CARGO_HEIGHT_INVALID);
}

TEST(CargoSafetyEvaluator, InvalidObstacleEvidenceNeverBecomesLevel2Warning) {
    CargoSafetyEvaluator evaluator;

    CargoSafetyInput stale = baseInput();
    stale.obstacle_cloud_age_sec = 1.0;
    const CargoSafetyResult stale_result = evaluator.evaluate(stale);
    EXPECT_EQ(stale_result.warning_code, 0U);
    EXPECT_EQ(stale_result.fault,
              CargoSafetyFault::OBSTACLE_EVIDENCE_INVALID);

    CargoSafetyInput coverage = baseInput();
    coverage.obstacle_roi_coverage_ratio = 0.0F;
    const CargoSafetyResult coverage_result = evaluator.evaluate(coverage);
    EXPECT_EQ(coverage_result.warning_code, 0U);
    EXPECT_EQ(coverage_result.fault,
              CargoSafetyFault::OBSTACLE_EVIDENCE_INVALID);

    CargoSafetyInput sparse = baseInput();
    auto sparse_cloud = mutableObstacleCloud(&sparse);

    for (int i = 0; i < 3; ++i) {
        sparse_cloud->push_back(
            pcl::PointXYZ(2.0F + 0.01F * i, 0.0F, 1.0F));
    }
    const CargoSafetyResult sparse_result = evaluator.evaluate(sparse);
    EXPECT_FALSE(sparse_result.input_valid);
    EXPECT_FALSE(sparse_result.warning_valid);
    EXPECT_EQ(sparse_result.warning_code, 0U);
    EXPECT_EQ(sparse_result.fault,
              CargoSafetyFault::OBSTACLE_EVIDENCE_INVALID);
    EXPECT_EQ(sparse_result.reason, "sparse_obstacle_returns");
}

TEST(CargoSafetyEvaluator, SufficientScatteredCandidatesAreInvalidEvidence) {
    CargoSafetyInput input = baseInput();
    auto cloud = mutableObstacleCloud(&input);

    for (int i = 0; i < 5; ++i) {
        cloud->push_back(pcl::PointXYZ(
            1.0F + 0.50F * static_cast<float>(i), 0.0F, 1.0F));
    }

    const CargoSafetyResult result = CargoSafetyEvaluator().evaluate(input);
    EXPECT_FALSE(result.input_valid);
    EXPECT_FALSE(result.warning_valid);
    EXPECT_EQ(result.warning_code, 0U);
    EXPECT_EQ(result.fault, CargoSafetyFault::OBSTACLE_EVIDENCE_INVALID);
    EXPECT_EQ(result.reason, "obstacle_clusters_insufficient");
    EXPECT_FALSE(result.has_cluster_evidence);
    EXPECT_EQ(result.evaluated_cluster_count, 0U);
}

TEST(CargoSafetyEvaluator, ValidObservationWithNoObstacleIsClear) {
    const CargoSafetyResult result =
        CargoSafetyEvaluator().evaluate(baseInput());
    EXPECT_TRUE(result.input_valid);
    EXPECT_TRUE(result.warning_valid);
    EXPECT_EQ(result.warning_code, CargoSafetyEvaluator::kSafeCode);
    EXPECT_EQ(result.fault, CargoSafetyFault::NONE);
    EXPECT_FALSE(result.has_cluster_evidence);
    EXPECT_EQ(result.evaluated_cluster_count, 0U);
    EXPECT_EQ(result.reason, "clear_no_external_obstacle");
}

TEST(CargoSafetyEvaluator, EvaluatorNeverSilentlyRemovesRuntimeInput) {
    CargoSafetyInput input = baseInput();
    auto cloud = mutableObstacleCloud(&input);

    for (int i = 0; i < 8; ++i) {
        cloud->push_back(pcl::PointXYZ(
            0.01F * static_cast<float>(i), 0.0F,
            2.10F + 0.002F * static_cast<float>(i)));
    }
    const CargoSafetyResult result = CargoSafetyEvaluator().evaluate(input);
    EXPECT_TRUE(result.input_valid);
    EXPECT_TRUE(result.warning_valid);
    EXPECT_EQ(result.warning_code, CargoSafetyEvaluator::kLevel1Code);
    EXPECT_EQ(result.fault, CargoSafetyFault::NONE);
    EXPECT_TRUE(result.has_cluster_evidence);
    EXPECT_EQ(result.self_cargo_points_removed, 0U);
}

TEST(CargoSafetyEvaluator, InvalidConfigAndInputAreInternalErrors) {
    CargoSafetyConfig invalid_config;
    invalid_config.level2_distance_m = 2.0F;
    CargoSafetyEvaluator invalid_evaluator(invalid_config);
    const CargoSafetyResult config_result = invalid_evaluator.evaluate(baseInput());
    EXPECT_EQ(config_result.warning_code, 0U);
    EXPECT_EQ(config_result.fault, CargoSafetyFault::INTERNAL_ERROR);

    CargoSafetyEvaluator evaluator;
    CargoSafetyInput invalid_input = baseInput();
    invalid_input.footprint_base.length_m = 0.0F;
    const CargoSafetyResult input_result = evaluator.evaluate(invalid_input);
    EXPECT_EQ(input_result.warning_code, 0U);
    EXPECT_EQ(input_result.fault, CargoSafetyFault::INTERNAL_ERROR);
}

TEST(CargoSafetyEvaluator, Level1HasPriorityAcrossClusters) {
    CargoSafetyInput mixed = baseInput();
    addCluster(&mixed, 4.0F, 1.20F, 0.0F);
    addCluster(&mixed, 2.0F, 1.20F, 0.5F);
    const CargoSafetyResult mixed_result =
        CargoSafetyEvaluator().evaluate(mixed);
    ASSERT_EQ(mixed_result.evaluated_cluster_count, 2U);
    EXPECT_EQ(mixed_result.warning_code, CargoSafetyEvaluator::kLevel1Code);

    CargoSafetyInput outer = baseInput();
    addCluster(&outer, 4.0F, 1.20F);
    EXPECT_EQ(CargoSafetyEvaluator().evaluate(outer).warning_code,
              CargoSafetyEvaluator::kLevel2Code);

    CargoSafetyInput clear = baseInput();
    addCluster(&clear, 2.0F, 0.20F, 0.0F);
    addCluster(&clear, 4.0F, 0.20F, 0.5F);
    EXPECT_EQ(CargoSafetyEvaluator().evaluate(clear).warning_code,
              CargoSafetyEvaluator::kSafeCode);
}

TEST(CargoSafetyEvaluator, EntireClusterAboveCargoDoesNotWarn) {
    CargoSafetyInput input = baseInput();
    addCluster(&input, 2.0F, 4.20F);
    const CargoSafetyResult result = CargoSafetyEvaluator().evaluate(input);
    ASSERT_TRUE(result.warning_valid) << result.reason;
    EXPECT_EQ(result.warning_code, CargoSafetyEvaluator::kSafeCode);
    ASSERT_TRUE(result.has_cluster_evidence);
    EXPECT_TRUE(result.most_dangerous_cluster.entirely_above_cargo);
}

TEST(CargoSafetyEvaluator, VerticallyContinuousHighWallStillWarns) {
    CargoSafetyInput input = baseInput();
    auto cloud = mutableObstacleCloud(&input);
    for (int index = 0; index < 32; ++index) {
        cloud->push_back(pcl::PointXYZ(
            2.0F + 0.003F * static_cast<float>(index % 3),
            0.003F * static_cast<float>(index % 4),
            0.20F + 0.10F * static_cast<float>(index)));
    }
    const CargoSafetyResult result = CargoSafetyEvaluator().evaluate(input);
    ASSERT_TRUE(result.warning_valid) << result.reason;
    EXPECT_EQ(result.warning_code, CargoSafetyEvaluator::kLevel1Code);
    ASSERT_TRUE(result.has_cluster_evidence);
    EXPECT_FALSE(result.most_dangerous_cluster.entirely_above_cargo);
    EXPECT_GE(result.most_dangerous_cluster.vertical_continuity_ratio,
              0.45F);
}

TEST(CargoSafetyEvaluator, NeverExcludesObstacleBelowFusedBottom) {
    CargoSafetyInput input = baseInput();
    auto cloud = mutableObstacleCloud(&input);

    for (int i = 0; i < 8; ++i) {
        cloud->push_back(pcl::PointXYZ(
            0.01F * static_cast<float>(i), 0.0F,
            1.70F + 0.002F * static_cast<float>(i)));
    }
    const CargoSafetyResult result = CargoSafetyEvaluator().evaluate(input);
    EXPECT_TRUE(result.warning_valid);
    EXPECT_EQ(result.self_cargo_points_removed, 0U);
    EXPECT_EQ(result.warning_code, CargoSafetyEvaluator::kLevel1Code);
}

TEST(CargoSafetyEvaluator, RotatedFootprintUsesObbDistanceWithoutSelfRemoval) {
    CargoSafetyInput input = baseInput();
    input.footprint_base.length_m = 2.0F;
    input.footprint_base.width_m = 0.6F;
    input.footprint_base.yaw_base_rad =
        0.5F * 3.14159265358979323846F;
    auto cloud = mutableObstacleCloud(&input);
    for (int i = 0; i < 8; ++i) {
        cloud->push_back(pcl::PointXYZ(
            0.0F, 0.70F + 0.005F * static_cast<float>(i), 2.10F));
    }
    const CargoSafetyResult result = CargoSafetyEvaluator().evaluate(input);
    EXPECT_TRUE(result.warning_valid);
    EXPECT_EQ(result.self_cargo_points_removed, 0U);
    EXPECT_EQ(result.warning_code, CargoSafetyEvaluator::kLevel1Code);
}

}  // namespace
}  // namespace ndt_slam
