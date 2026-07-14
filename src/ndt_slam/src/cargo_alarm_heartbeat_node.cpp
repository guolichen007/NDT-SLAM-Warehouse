#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace cargo_alarm {

struct StatusContractInput {
    bool schema_valid = false;
    bool valid = false;
    bool warning_valid = false;
    int requested_code = 0;
    int warning_code = 0;
    int fault_code = 0;
    std::uint32_t fault_mask = 0U;
    bool localization_valid = false;
    int hook_signal_role = 1;
    bool hook_signal_valid = false;
    bool hook_signal_conflict = false;
    int hook_load_state = 0;
    bool no_cargo_confirmed = false;
    bool cargo_valid = false;
    bool obstacle_valid = false;
    std::uint32_t obstacle_count = 0U;
    double nearest_obstacle_distance_m =
        std::numeric_limits<double>::quiet_NaN();
    double obstacle_top_z_map = std::numeric_limits<double>::quiet_NaN();
    double obstacle_uncertainty_m =
        std::numeric_limits<double>::quiet_NaN();
    double conservative_vertical_clearance_m =
        std::numeric_limits<double>::quiet_NaN();
};

struct StatusContractResult {
    int code = 35;
    bool valid = false;
    bool clear_without_obstacle_geometry = false;
    const char* reason = "schema_mismatch";
};

struct StatusContractConfig {
    double level1_distance_m = 3.0;
    double level2_distance_m = 5.0;
    double minimum_vertical_clearance_m = 0.80;
};

bool isValidStatusContractConfig(const StatusContractConfig& config) {
    return std::isfinite(config.level1_distance_m) &&
        std::isfinite(config.level2_distance_m) &&
        std::isfinite(config.minimum_vertical_clearance_m) &&
        config.level1_distance_m > 0.0 &&
        config.level2_distance_m > config.level1_distance_m &&
        config.minimum_vertical_clearance_m >= 0.0;
}

class AlarmStateMachine {
public:
    static constexpr int kClear = 14;
    static constexpr int kLevel1Warning = 17;
    static constexpr int kLevel2Warning = 18;
    static constexpr int kSystemNotReady = 30;
    static constexpr int kLocalizationInvalid = 31;
    static constexpr int kGravityInvalid = 32;
    static constexpr int kCargoInvalid = 33;
    static constexpr int kObstacleInvalid = 34;
    static constexpr int kInternalError = 35;
    static constexpr int kHookEmpty = 2;
    static constexpr int kHookLoaded = 3;
    static constexpr int kHookRoleDisabled = 0;
    static constexpr int kHookRoleRequired = 1;
    static constexpr int kHookRoleAuxiliary = 2;

    struct Config {
        int confirm_frames = 2;
        double level1_exit_distance_m = 3.20;
        double level2_exit_distance_m = 5.20;
        double clearance_exit_m = 0.90;
        double immediate_clearance_m = 0.50;
        double clear_delay_sec = 0.50;
    };

    struct Result {
        int code = kSystemNotReady;
        bool changed = false;
        const char* reason = "startup_not_ready";
    };

    explicit AlarmStateMachine(double stale_timeout_sec)
        : AlarmStateMachine(stale_timeout_sec, Config{}) {}

    AlarmStateMachine(double stale_timeout_sec, const Config& config)
        : stale_timeout_sec_(sanitizePositive(stale_timeout_sec, 0.8)),
          confirm_frames_(std::max(1, config.confirm_frames)),
          level1_exit_distance_m_(sanitizePositive(
              config.level1_exit_distance_m, 3.20)),
          level2_exit_distance_m_(sanitizePositive(
              config.level2_exit_distance_m, 5.20)),
          clearance_exit_m_(sanitizePositive(config.clearance_exit_m, 0.90)),
          immediate_clearance_m_(sanitizeNonNegative(
              config.immediate_clearance_m, 0.50)),
          clear_delay_sec_(sanitizeNonNegative(config.clear_delay_sec, 0.50)) {}

    int currentCode() const { return current_code_; }

    Result ingest(int requested_code,
                  double wall_now_sec,
                  double source_stamp_sec,
                  double distance_m = std::numeric_limits<double>::infinity(),
                  double clearance_m = std::numeric_limits<double>::infinity(),
                  bool clear_without_obstacle_geometry = false) {
        if (!observeWallClock(wall_now_sec)) {
            return forceCode(kSystemNotReady, "wall_time_rollback");
        }

        has_status_ = true;
        last_receipt_wall_sec_ = wall_now_sec;

        if (!std::isfinite(source_stamp_sec) || source_stamp_sec <= 0.0) {
            return forceCode(kInternalError, "invalid_source_stamp");
        }
        if (has_source_stamp_ &&
            source_stamp_sec + kTimeEpsilonSec < last_source_stamp_sec_) {
            last_source_stamp_sec_ = source_stamp_sec;
            last_source_progress_wall_sec_ = wall_now_sec;
            has_source_stamp_ = true;
            return forceCode(kSystemNotReady, "source_time_rollback");
        }
        const bool source_stamp_advanced = !has_source_stamp_ ||
            source_stamp_sec > last_source_stamp_sec_ + kTimeEpsilonSec;
        if (source_stamp_advanced) {
            last_source_stamp_sec_ = source_stamp_sec;
            last_source_progress_wall_sec_ = wall_now_sec;
        }
        has_source_stamp_ = true;

        if (wall_now_sec - last_source_progress_wall_sec_ + kTimeEpsilonSec >=
            stale_timeout_sec_) {
            return forceCode(kSystemNotReady, "source_stamp_stale");
        }
        if (!isAllowedCode(requested_code)) {
            return forceCode(kInternalError, "unknown_status_code");
        }

        last_candidate_code_ = requested_code;
        last_distance_m_ = distance_m;
        last_clearance_m_ = clearance_m;
        last_clear_without_obstacle_geometry_ =
            clear_without_obstacle_geometry;
        return applyCandidate(requested_code, wall_now_sec, distance_m,
                              clearance_m, clear_without_obstacle_geometry,
                              source_stamp_advanced,
                              "fresh_status");
    }

    Result tick(double wall_now_sec) {
        if (!observeWallClock(wall_now_sec)) {
            return forceCode(kSystemNotReady, "wall_time_rollback");
        }
        if (!has_status_) {
            return forceCode(kSystemNotReady, "no_status");
        }
        if (wall_now_sec - last_receipt_wall_sec_ + kTimeEpsilonSec >=
            stale_timeout_sec_) {
            return forceCode(kSystemNotReady, "status_stale");
        }
        if (wall_now_sec - last_source_progress_wall_sec_ + kTimeEpsilonSec >=
            stale_timeout_sec_) {
            return forceCode(kSystemNotReady, "source_stamp_stale");
        }
        return applyCandidate(last_candidate_code_, wall_now_sec,
                              last_distance_m_, last_clearance_m_,
                              last_clear_without_obstacle_geometry_, false,
                              "heartbeat");
    }

    static bool isAllowedCode(int code) {
        return code == kClear || code == kLevel1Warning ||
               code == kLevel2Warning ||
               (code >= kSystemNotReady && code <= kInternalError);
    }

private:
    static constexpr double kTimeEpsilonSec = 1e-6;

    static double sanitizePositive(double value, double fallback) {
        return std::isfinite(value) && value > 0.0 ? value : fallback;
    }

    static double sanitizeNonNegative(double value, double fallback) {
        return std::isfinite(value) && value >= 0.0 ? value : fallback;
    }

    static bool isFaultCode(int code) {
        return code >= kSystemNotReady && code <= kInternalError;
    }

    bool observeWallClock(double wall_now_sec) {
        if (!std::isfinite(wall_now_sec) || wall_now_sec < 0.0) return false;
        if (has_wall_observation_ &&
            wall_now_sec + kTimeEpsilonSec < last_observed_wall_sec_) {
            last_observed_wall_sec_ = wall_now_sec;
            return false;
        }
        has_wall_observation_ = true;
        last_observed_wall_sec_ = wall_now_sec;
        return true;
    }

    Result forceCode(int code, const char* reason) {
        const bool changed = current_code_ != code;
        current_code_ = code;
        last_candidate_code_ = code;
        pending_candidate_code_ = 0;
        pending_candidate_frames_ = 0;
        clear_pending_ = false;
        return {current_code_, changed, reason};
    }

    bool candidateConfirmed(int candidate, bool fresh_source_evidence) {
        if (pending_candidate_code_ != candidate) {
            pending_candidate_code_ = candidate;
            pending_candidate_frames_ = fresh_source_evidence ? 1 : 0;
        } else if (fresh_source_evidence) {
            ++pending_candidate_frames_;
        }
        return pending_candidate_frames_ >= confirm_frames_;
    }

    void resetCandidateConfirmation() {
        pending_candidate_code_ = 0;
        pending_candidate_frames_ = 0;
    }

    Result applyCandidate(int candidate,
                          double wall_now_sec,
                          double distance_m,
                          double clearance_m,
                          bool clear_without_obstacle_geometry,
                          bool fresh_source_evidence,
                          const char* reason) {
        if (isFaultCode(candidate)) {
            return forceCode(candidate, reason);
        }

        if (candidate == kLevel1Warning || candidate == kLevel2Warning) {
            clear_pending_ = false;
            if (pending_candidate_code_ == kClear) {
                resetCandidateConfirmation();
            }
            const bool severe = std::isfinite(clearance_m) &&
                clearance_m < immediate_clearance_m_;
            const bool level2_to_level1 =
                current_code_ == kLevel2Warning && candidate == kLevel1Warning;
            if (candidate == current_code_) {
                resetCandidateConfirmation();
                return {current_code_, false, reason};
            }

            if (current_code_ == kLevel1Warning &&
                candidate == kLevel2Warning) {
                const bool exited_level1 = std::isfinite(distance_m) &&
                    distance_m > level1_exit_distance_m_;
                if (!exited_level1 ||
                    !candidateConfirmed(candidate, fresh_source_evidence)) {
                    return {current_code_, false, "level1_exit_pending"};
                }
            } else if (!severe && !level2_to_level1 &&
                       !candidateConfirmed(candidate, fresh_source_evidence)) {
                return {current_code_, false, "warning_confirm_pending"};
            }

            resetCandidateConfirmation();
            const bool changed = current_code_ != candidate;
            current_code_ = candidate;
            return {current_code_, changed, severe
                ? "immediate_low_clearance" : reason};
        }

        if (candidate == kClear && current_code_ != kClear) {
            const bool requires_clear_confirmation = current_code_ != kClear;
            bool geometry_exited = clear_without_obstacle_geometry ||
                isFaultCode(current_code_);
            if (current_code_ == kLevel1Warning &&
                !clear_without_obstacle_geometry) {
                geometry_exited =
                    (std::isfinite(distance_m) &&
                     distance_m > level1_exit_distance_m_) ||
                    (std::isfinite(clearance_m) &&
                     clearance_m >= clearance_exit_m_);
            } else if (current_code_ == kLevel2Warning &&
                       !clear_without_obstacle_geometry) {
                geometry_exited =
                    (std::isfinite(distance_m) &&
                     distance_m > level2_exit_distance_m_) ||
                    (std::isfinite(clearance_m) &&
                     clearance_m >= clearance_exit_m_);
            }
            if (!geometry_exited) {
                clear_pending_ = false;
                resetCandidateConfirmation();
                return {current_code_, false, "clear_hysteresis_hold"};
            }
            if (requires_clear_confirmation && !clear_pending_ &&
                !candidateConfirmed(kClear, fresh_source_evidence)) {
                return {current_code_, false, "clear_confirm_pending"};
            }
            if (!clear_pending_) {
                resetCandidateConfirmation();
                clear_pending_ = true;
                clear_pending_since_wall_sec_ = wall_now_sec;
                return {current_code_, false, "clear_delay_started"};
            }
            if (wall_now_sec - clear_pending_since_wall_sec_ + kTimeEpsilonSec <
                clear_delay_sec_) {
                return {current_code_, false, "clear_delay_pending"};
            }
            current_code_ = kClear;
            clear_pending_ = false;
            return {current_code_, true, "clear_delay_satisfied"};
        }

        resetCandidateConfirmation();
        clear_pending_ = false;
        return {current_code_, false, reason};
    }

    const double stale_timeout_sec_;
    const int confirm_frames_;
    const double level1_exit_distance_m_;
    const double level2_exit_distance_m_;
    const double clearance_exit_m_;
    const double immediate_clearance_m_;
    const double clear_delay_sec_;
    int current_code_ = kSystemNotReady;
    int last_candidate_code_ = kSystemNotReady;
    int pending_candidate_code_ = 0;
    int pending_candidate_frames_ = 0;
    bool has_status_ = false;
    bool has_wall_observation_ = false;
    bool has_source_stamp_ = false;
    bool clear_pending_ = false;
    bool last_clear_without_obstacle_geometry_ = false;
    double last_receipt_wall_sec_ = 0.0;
    double last_observed_wall_sec_ = 0.0;
    double last_source_stamp_sec_ = 0.0;
    double last_source_progress_wall_sec_ = 0.0;
    double clear_pending_since_wall_sec_ = 0.0;
    double last_distance_m_ = std::numeric_limits<double>::infinity();
    double last_clearance_m_ = std::numeric_limits<double>::infinity();
};

StatusContractResult validateStatusContract(
    const StatusContractInput& input,
    const StatusContractConfig& config = StatusContractConfig()) {
    using State = AlarmStateMachine;
    if (!isValidStatusContractConfig(config)) {
        return {State::kInternalError, false, false,
                "status_contract_config_invalid"};
    }
    if (!input.schema_valid) {
        return {State::kInternalError, false, false, "schema_mismatch"};
    }
    const bool hook_role_valid =
        input.hook_signal_role == State::kHookRoleDisabled ||
        input.hook_signal_role == State::kHookRoleRequired ||
        input.hook_signal_role == State::kHookRoleAuxiliary;
    if (!hook_role_valid) {
        return {State::kInternalError, false, false, "hook_role_invalid"};
    }
    const bool gravity_required =
        input.hook_signal_role == State::kHookRoleRequired;
    const bool warning_code_valid = input.warning_code == 0 ||
        input.warning_code == State::kClear ||
        input.warning_code == State::kLevel1Warning ||
        input.warning_code == State::kLevel2Warning;
    const bool fault_code_valid = input.fault_code == 0 ||
        (input.fault_code >= State::kSystemNotReady &&
         input.fault_code <= State::kInternalError);
    if (!warning_code_valid || !fault_code_valid ||
        !State::isAllowedCode(input.requested_code)) {
        return {State::kInternalError, false, false, "unknown_status_code"};
    }

    if (input.fault_code != 0) {
        std::uint32_t required_mask = 0U;
        switch (input.fault_code) {
            case State::kSystemNotReady: required_mask = 1U; break;
            case State::kLocalizationInvalid: required_mask = 2U; break;
            case State::kGravityInvalid: required_mask = 4U; break;
            case State::kCargoInvalid: required_mask = 8U; break;
            case State::kObstacleInvalid: required_mask = 16U; break;
            case State::kInternalError: required_mask = 32U; break;
            default: break;
        }
        if (input.requested_code != input.fault_code ||
            input.warning_valid || input.warning_code != 0 ||
            required_mask == 0U ||
            (input.fault_mask & required_mask) == 0U) {
            return {State::kInternalError, false, false,
                    "fault_contract_mismatch"};
        }
        if (input.fault_code == State::kGravityInvalid &&
            !gravity_required) {
            return {State::kInternalError, false, false,
                    "auxiliary_gravity_fault_forbidden"};
        }
        return {input.fault_code, true, false, "fault_status"};
    }

    if (input.fault_mask != 0U || !input.warning_valid || !input.valid ||
        input.requested_code != input.warning_code ||
        input.warning_code == 0) {
        return {State::kInternalError, false, false,
                "warning_contract_mismatch"};
    }

    const bool hook_supports_empty = !gravity_required ||
        (input.hook_signal_valid &&
         input.hook_load_state == State::kHookEmpty);
    const bool hook_supports_loaded = !gravity_required ||
        (input.hook_signal_valid &&
         input.hook_load_state == State::kHookLoaded);
    const bool safe_empty = input.localization_valid &&
        hook_supports_empty &&
        input.no_cargo_confirmed && !input.cargo_valid;
    const bool valid_loaded = input.localization_valid &&
        hook_supports_loaded &&
        !input.no_cargo_confirmed && input.cargo_valid && input.obstacle_valid;
    const bool cluster_geometry_valid = input.obstacle_count > 0U &&
        std::isfinite(input.nearest_obstacle_distance_m) &&
        input.nearest_obstacle_distance_m >= 0.0 &&
        std::isfinite(input.obstacle_top_z_map) &&
        std::isfinite(input.obstacle_uncertainty_m) &&
        input.obstacle_uncertainty_m >= 0.0 &&
        std::isfinite(input.conservative_vertical_clearance_m);

    if (input.warning_code == State::kClear) {
        if (safe_empty) {
            return {State::kClear, true, true, "clear_status"};
        }
        if (!valid_loaded) {
            return {State::kInternalError, false, false,
                    "clear_contract_mismatch"};
        }
        if (input.obstacle_count == 0U) {
            return {State::kClear, true, true, "clear_status"};
        }
        if (!cluster_geometry_valid) {
            return {State::kInternalError, false, false,
                    "clear_contract_mismatch"};
        }
        const bool safe_geometry =
            input.conservative_vertical_clearance_m >=
                config.minimum_vertical_clearance_m ||
            input.nearest_obstacle_distance_m > config.level2_distance_m;
        if (!safe_geometry) {
            return {State::kInternalError, false, false,
                    "clear_geometry_mismatch"};
        }
        return {State::kClear, true, false, "clear_status"};
    }
    const bool level1_geometry =
        input.nearest_obstacle_distance_m <= config.level1_distance_m &&
        input.conservative_vertical_clearance_m <
            config.minimum_vertical_clearance_m;
    const bool level2_geometry =
        input.nearest_obstacle_distance_m > config.level1_distance_m &&
        input.nearest_obstacle_distance_m <= config.level2_distance_m &&
        input.conservative_vertical_clearance_m <
            config.minimum_vertical_clearance_m;
    if (!valid_loaded || !cluster_geometry_valid ||
        (input.warning_code == State::kLevel1Warning && !level1_geometry) ||
        (input.warning_code == State::kLevel2Warning && !level2_geometry)) {
        return {State::kInternalError, false, false,
                "warning_geometry_mismatch"};
    }
    return {input.warning_code, true, false, "hazard_status"};
}

}  // namespace cargo_alarm

#ifndef CARGO_ALARM_HEARTBEAT_STATE_MACHINE_ONLY

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <lidar_slam2_msgs/CargoSafetyStatus.h>

namespace cargo_alarm {

class AlarmHeartbeatNode {
public:
    AlarmHeartbeatNode()
        : nh_(),
          pnh_("~"),
          heartbeat_hz_(readPositiveParam("heartbeat_hz", 5.0)),
          stale_timeout_sec_(readPositiveParam("stale_timeout_sec", 0.8)),
          warning_config_(readWarningConfig()),
          contract_config_(readStatusContractConfig()),
          state_machine_(stale_timeout_sec_, warning_config_) {
        pnh_.param<std::string>("status_topic", status_topic_,
                                "/cargo_avoidance/safety_status");
        pnh_.param<std::string>("status_code_topic", status_code_topic_,
                                "/cargo_avoidance/status_code");
        pnh_.param("publish_legacy_alarm_topic", publish_legacy_alarm_topic_,
                   false);
        pnh_.param<std::string>("legacy_alarm_topic", legacy_alarm_topic_,
                                "/cargo_avoidance/alarm_code");

        int status_queue_size = 1;
        pnh_.param("status_queue_size", status_queue_size, status_queue_size);
        if (status_queue_size != 1) {
            ROS_WARN("[CargoAlarmHeartbeat] status_queue_size=%d is unsafe; using 1",
                     status_queue_size);
            status_queue_size = 1;
        }

        status_code_pub_ = nh_.advertise<std_msgs::Int32>(
            status_code_topic_, 1, true);
        if (publish_legacy_alarm_topic_) {
            legacy_alarm_pub_ = nh_.advertise<std_msgs::Int32>(
                legacy_alarm_topic_, 1, true);
        }
        status_sub_ = nh_.subscribe(status_topic_, status_queue_size,
                                    &AlarmHeartbeatNode::statusCallback, this);

        publishCode(AlarmStateMachine::kSystemNotReady,
                    "startup_not_ready", true);
        heartbeat_timer_ = nh_.createWallTimer(
            ros::WallDuration(1.0 / heartbeat_hz_),
            &AlarmHeartbeatNode::heartbeatCallback, this);

        ROS_INFO("[CargoAlarmHeartbeat] status=%s status_code=%s "
                 "heartbeat=%.2fHz stale=%.3fs legacy=%d",
                 status_topic_.c_str(), status_code_topic_.c_str(),
                 heartbeat_hz_, stale_timeout_sec_,
                 publish_legacy_alarm_topic_ ? 1 : 0);
    }

private:
    double readPositiveParam(const std::string& name, double fallback) {
        double value = fallback;
        pnh_.param(name, value, fallback);
        if (!std::isfinite(value) || value <= 0.0) {
            ROS_ERROR("[CargoAlarmHeartbeat] ~%s=%.6f invalid; using %.6f",
                      name.c_str(), value, fallback);
            return fallback;
        }
        return value;
    }

    double readNonNegativeParam(const std::string& name, double fallback) {
        double value = fallback;
        pnh_.param(name, value, fallback);
        if (!std::isfinite(value) || value < 0.0) {
            ROS_ERROR("[CargoAlarmHeartbeat] ~%s=%.6f invalid; using %.6f",
                      name.c_str(), value, fallback);
            return fallback;
        }
        return value;
    }

    AlarmStateMachine::Config readWarningConfig() {
        AlarmStateMachine::Config config;
        pnh_.param("confirm_frames", config.confirm_frames, 2);
        config.level1_exit_distance_m =
            readPositiveParam("level1_exit_distance_m", 3.20);
        config.level2_exit_distance_m =
            readPositiveParam("level2_exit_distance_m", 5.20);
        config.clearance_exit_m =
            readPositiveParam("clearance_exit_m", 0.90);
        config.immediate_clearance_m =
            readNonNegativeParam("immediate_clearance_m", 0.50);
        config.clear_delay_sec =
            readNonNegativeParam("clear_delay_sec", 0.50);
        const bool valid = config.confirm_frames == 2 &&
            std::abs(config.level1_exit_distance_m - 3.20) <= 1.0e-6 &&
            std::abs(config.level2_exit_distance_m - 5.20) <= 1.0e-6 &&
            std::abs(config.clearance_exit_m - 0.90) <= 1.0e-6 &&
            std::abs(config.immediate_clearance_m - 0.50) <= 1.0e-6 &&
            std::abs(config.clear_delay_sec - 0.50) <= 1.0e-6;
        if (!valid) {
            ROS_ERROR("[CargoAlarmHeartbeat] invalid warning_state; "
                      "restoring confirm=2 exit=(3.20,5.20,0.90) "
                      "immediate=0.50 clear_delay=0.50");
            config = AlarmStateMachine::Config();
        }
        return config;
    }

    StatusContractConfig readStatusContractConfig() {
        StatusContractConfig config;
        pnh_.param("level1_distance_m", config.level1_distance_m, 3.0);
        pnh_.param("level2_distance_m", config.level2_distance_m, 5.0);
        pnh_.param("minimum_vertical_clearance_m",
                   config.minimum_vertical_clearance_m, 0.80);
        if (!isValidStatusContractConfig(config)) {
            ROS_ERROR("[CargoAlarmHeartbeat] invalid status contract "
                      "entry=(%.6f,%.6f,%.6f); restoring (3.0,5.0,0.80)",
                      config.level1_distance_m, config.level2_distance_m,
                      config.minimum_vertical_clearance_m);
            config = StatusContractConfig();
        }
        return config;
    }

    void statusCallback(const lidar_slam2_msgs::CargoSafetyStatus::ConstPtr& msg) {
        static_assert(lidar_slam2_msgs::CargoSafetyStatus::SCHEMA_VERSION == 4,
                      "CargoSafetyStatus schema v4 is required");
        static_assert(
            lidar_slam2_msgs::CargoSafetyStatus::HOOK_ROLE_DISABLED ==
                AlarmStateMachine::kHookRoleDisabled &&
            lidar_slam2_msgs::CargoSafetyStatus::HOOK_ROLE_REQUIRED ==
                AlarmStateMachine::kHookRoleRequired &&
            lidar_slam2_msgs::CargoSafetyStatus::HOOK_ROLE_AUXILIARY ==
                AlarmStateMachine::kHookRoleAuxiliary,
            "Heartbeat hook-role constants do not match the message schema");
        StatusContractInput input;
        input.schema_valid = msg->schema_version ==
            lidar_slam2_msgs::CargoSafetyStatus::SCHEMA_VERSION;
        input.valid = msg->valid;
        input.warning_valid = msg->warning_valid;
        input.requested_code = msg->requested_alarm_code;
        input.warning_code = msg->warning_code;
        input.fault_code = msg->fault_code;
        input.fault_mask = msg->fault_mask;
        input.localization_valid = msg->localization_valid;
        input.hook_signal_role = msg->hook_signal_role;
        input.hook_signal_valid = msg->hook_signal_valid;
        input.hook_signal_conflict = msg->hook_signal_conflict;
        input.hook_load_state = msg->hook_load_state;
        input.no_cargo_confirmed = msg->no_cargo_confirmed;
        input.cargo_valid = msg->cargo_valid;
        input.obstacle_valid = msg->obstacle_valid;
        input.obstacle_count = msg->obstacle_count;
        input.nearest_obstacle_distance_m =
            msg->nearest_obstacle_distance_m;
        input.obstacle_top_z_map = msg->obstacle_top_z_map;
        input.obstacle_uncertainty_m = msg->obstacle_uncertainty_m;
        input.conservative_vertical_clearance_m =
            msg->conservative_vertical_clearance_m;
        const StatusContractResult contract =
            validateStatusContract(input, contract_config_);

        last_status_ = *msg;
        if (!contract.valid) last_status_.reason = contract.reason;
        has_last_status_ = true;
        const AlarmStateMachine::Result result = state_machine_.ingest(
            contract.code, ros::WallTime::now().toSec(),
            msg->header.stamp.toSec(), msg->nearest_obstacle_distance_m,
            msg->conservative_vertical_clearance_m,
            contract.clear_without_obstacle_geometry);
        if (result.changed) publishCode(result.code, result.reason, true);
    }

    void heartbeatCallback(const ros::WallTimerEvent& event) {
        const AlarmStateMachine::Result result =
            state_machine_.tick(event.current_real.toSec());
        publishCode(result.code, result.reason, result.changed);
    }

    void publishCode(int code, const char* reason, bool log_change) {
        std_msgs::Int32 message;
        message.data = code;
        status_code_pub_.publish(message);
        if (publish_legacy_alarm_topic_) legacy_alarm_pub_.publish(message);

        if (!log_change) {
            ROS_DEBUG("[CargoAlarmHeartbeat] code=%d reason=%s", code, reason);
            return;
        }
        if (code == AlarmStateMachine::kClear) {
            ROS_INFO("[SAFETY] code=14 state=CLEAR "
                     "localization_valid=%d gravity_valid=%d "
                     "cargo_valid=%d obstacle_valid=%d",
                     has_last_status_ && last_status_.localization_valid,
                     has_last_status_ && last_status_.hook_signal_valid,
                     has_last_status_ && last_status_.cargo_valid,
                     has_last_status_ && last_status_.obstacle_valid);
        } else if (code == AlarmStateMachine::kLevel1Warning ||
                   code == AlarmStateMachine::kLevel2Warning) {
            ROS_WARN("[SAFETY_WARN] code=%d level=%d distance=%.2f "
                     "clearance=%.2f cargo_bottom=%.2f obstacle_top=%.2f "
                     "track=%u confidence=%.2f reason=%s",
                     code, code == AlarmStateMachine::kLevel1Warning ? 1 : 2,
                     last_status_.nearest_obstacle_distance_m,
                     last_status_.conservative_vertical_clearance_m,
                     last_status_.cargo_bottom_z_map,
                     last_status_.obstacle_top_z_map,
                     last_status_.cargo_track_id, last_status_.confidence,
                     last_status_.reason.c_str());
        } else {
            const bool message_reason = std::string(reason) == "fresh_status" ||
                std::string(reason) == "heartbeat";
            const std::string fault_reason = has_last_status_ && message_reason
                ? last_status_.reason : std::string(reason);
            ROS_ERROR("[SAFETY_FAULT] code=%d reason=%s",
                      code, fault_reason.c_str());
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber status_sub_;
    ros::Publisher status_code_pub_;
    ros::Publisher legacy_alarm_pub_;
    ros::WallTimer heartbeat_timer_;
    std::string status_topic_;
    std::string status_code_topic_;
    std::string legacy_alarm_topic_;
    bool publish_legacy_alarm_topic_ = false;
    const double heartbeat_hz_;
    const double stale_timeout_sec_;
    const AlarmStateMachine::Config warning_config_;
    const StatusContractConfig contract_config_;
    AlarmStateMachine state_machine_;
    lidar_slam2_msgs::CargoSafetyStatus last_status_;
    bool has_last_status_ = false;
};

}  // namespace cargo_alarm

int main(int argc, char** argv) {
    ros::init(argc, argv, "cargo_alarm_heartbeat");
    cargo_alarm::AlarmHeartbeatNode node;
    ros::spin();
    return 0;
}

#endif  // CARGO_ALARM_HEARTBEAT_STATE_MACHINE_ONLY
