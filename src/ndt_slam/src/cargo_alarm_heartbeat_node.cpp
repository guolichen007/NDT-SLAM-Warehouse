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
    std::uint32_t cargo_track_id = 0U;
    int evidence_state = 0;
    bool obstacle_valid = false;
    std::uint32_t obstacle_track_id = 0U;
    std::uint32_t obstacle_validated_streak = 0U;
    bool obstacle_large_geometry_valid = false;
    bool obstacle_provenance_valid = false;
    double confidence = 0.0;
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
    static constexpr int kEvidenceHazardCandidate = 3;

    struct Result {
        int code = kSystemNotReady;
        bool changed = false;
        const char* reason = "startup_not_ready";
    };

    explicit AlarmStateMachine(double stale_timeout_sec)
        : stale_timeout_sec_(sanitizePositive(stale_timeout_sec, 0.8)) {}

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

        (void)distance_m;
        (void)clearance_m;
        (void)clear_without_obstacle_geometry;
        if (!source_stamp_advanced) {
            return {current_code_, false, "duplicate_source_stamp"};
        }

        const bool changed = current_code_ != requested_code;
        current_code_ = requested_code;
        return {current_code_, changed, "fresh_status"};
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
        return {current_code_, false, "heartbeat"};
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
        return {current_code_, changed, reason};
    }

    const double stale_timeout_sec_;
    int current_code_ = kSystemNotReady;
    bool has_status_ = false;
    bool has_wall_observation_ = false;
    bool has_source_stamp_ = false;
    double last_receipt_wall_sec_ = 0.0;
    double last_observed_wall_sec_ = 0.0;
    double last_source_stamp_sec_ = 0.0;
    double last_source_progress_wall_sec_ = 0.0;
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
    const bool provisional_positive_loaded = input.localization_valid &&
        hook_supports_loaded && !input.no_cargo_confirmed &&
        !input.cargo_valid && input.obstacle_valid &&
        input.evidence_state == State::kEvidenceHazardCandidate;
    const bool provisional_track_contract =
        input.cargo_track_id > 0U &&
        input.obstacle_track_id > 0U &&
        input.obstacle_validated_streak > 0U &&
        input.obstacle_large_geometry_valid &&
        input.obstacle_provenance_valid &&
        std::isfinite(input.confidence) && input.confidence > 0.0;
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
    if ((!valid_loaded && !provisional_positive_loaded) ||
        (provisional_positive_loaded && !provisional_track_contract) ||
        !cluster_geometry_valid ||
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
          pending_error_sec_(readPositiveParam("pending_error_sec", 1.0)),
          pending_repeat_sec_(readPositiveParam("pending_repeat_sec", 10.0)),
          warning_repeat_sec_(readPositiveParam("warning_repeat_sec", 5.0)),
          contract_config_(readStatusContractConfig()),
          state_machine_(stale_timeout_sec_) {
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
        static_assert(lidar_slam2_msgs::CargoSafetyStatus::SCHEMA_VERSION == 6,
                      "CargoSafetyStatus schema v6 is required");
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
        input.cargo_track_id = msg->cargo_track_id;
        input.evidence_state = msg->evidence_state;
        input.obstacle_valid = msg->obstacle_valid;
        input.obstacle_track_id = msg->obstacle_track_id;
        input.obstacle_validated_streak =
            msg->obstacle_validated_streak;
        input.obstacle_large_geometry_valid =
            msg->obstacle_large_geometry_valid;
        input.obstacle_provenance_valid =
            msg->obstacle_provenance_valid;
        input.confidence = msg->confidence;
        input.obstacle_count = msg->obstacle_count;
        input.nearest_obstacle_distance_m =
            msg->nearest_obstacle_distance_m;
        input.obstacle_top_z_map = msg->obstacle_top_z_map;
        input.obstacle_uncertainty_m = msg->obstacle_uncertainty_m;
        input.conservative_vertical_clearance_m =
            msg->conservative_vertical_clearance_m;
        const StatusContractResult contract =
            validateStatusContract(input, contract_config_);

        lidar_slam2_msgs::CargoSafetyStatus accepted_status = *msg;
        if (!contract.valid) accepted_status.reason = contract.reason;
        const AlarmStateMachine::Result result = state_machine_.ingest(
            contract.code, ros::WallTime::now().toSec(),
            msg->header.stamp.toSec(), msg->nearest_obstacle_distance_m,
            msg->conservative_vertical_clearance_m,
            contract.clear_without_obstacle_geometry);
        const bool fresh_status = std::string(result.reason) == "fresh_status";
        const bool pending_episode_changed = fresh_status &&
            contract.code == AlarmStateMachine::kObstacleInvalid &&
            (!has_last_status_ ||
             last_status_.requested_alarm_code !=
                 accepted_status.requested_alarm_code ||
             last_status_.evidence_state != accepted_status.evidence_state ||
             last_status_.cargo_track_id != accepted_status.cargo_track_id);
        // A retransmitted source stamp is not a new geometry/evidence frame.
        // Keep the last accepted status so periodic 17/18 logs cannot pair a
        // retained alarm code with fields from a duplicate CLEAR message.
        if (!has_last_status_ || fresh_status) {
            last_status_ = accepted_status;
            has_last_status_ = true;
        }
        if (result.changed || pending_episode_changed) {
            publishCode(result.code, result.reason, true);
        }
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

        const double wall_now_sec = ros::WallTime::now().toSec();
        const std::string status_reason = has_last_status_
            ? last_status_.reason : std::string(reason);
        const bool pending_evidence =
            code == AlarmStateMachine::kObstacleInvalid &&
            has_last_status_ &&
            (last_status_.evidence_state ==
                 lidar_slam2_msgs::CargoSafetyStatus::
                     EVIDENCE_HAZARD_CANDIDATE ||
             last_status_.evidence_state ==
                 lidar_slam2_msgs::CargoSafetyStatus::
                     EVIDENCE_TRACK_CONFIRMATION_PENDING ||
             last_status_.evidence_state ==
                 lidar_slam2_msgs::CargoSafetyStatus::
                     EVIDENCE_SPARSE_PENDING ||
             last_status_.evidence_state ==
                 lidar_slam2_msgs::CargoSafetyStatus::
                     EVIDENCE_SOURCE_UNRESOLVED);
        if (pending_evidence) {
            if (!pending_active_ || log_change) {
                pending_active_ = true;
                pending_since_wall_sec_ = wall_now_sec;
                pending_error_reported_ = false;
            }
            const double pending_age = std::max(
                0.0, wall_now_sec - pending_since_wall_sec_);
            const bool persistent_due = pending_age >= pending_error_sec_ &&
                (!pending_error_reported_ ||
                 wall_now_sec - last_pending_log_wall_sec_ >=
                     pending_repeat_sec_);
            if (persistent_due) {
                ROS_WARN("[SAFETY_PENDING_SUMMARY] code=34 age=%.2f "
                         "evidence=%u cargo_track=%u obstacle_track=%u "
                         "validated=%u static=%u provenance=%u/%d "
                         "cell_overlap=%.2f reason=%s",
                         pending_age,
                         static_cast<unsigned int>(
                             last_status_.evidence_state),
                         last_status_.cargo_track_id,
                         last_status_.obstacle_track_id,
                         last_status_.obstacle_validated_streak,
                         last_status_.obstacle_static_provenance_streak,
                         static_cast<unsigned int>(
                             last_status_.obstacle_provenance_type),
                         last_status_.obstacle_provenance_valid ? 1 : 0,
                         last_status_.obstacle_track_cell_overlap,
                         status_reason.c_str());
                pending_error_reported_ = true;
                last_pending_log_wall_sec_ = wall_now_sec;
            } else if (log_change) {
                ROS_WARN("[SAFETY_PENDING] code=34 evidence=%u "
                         "cargo_track=%u obstacle_track=%u reason=%s",
                         static_cast<unsigned int>(
                             last_status_.evidence_state),
                         last_status_.cargo_track_id,
                         last_status_.obstacle_track_id,
                         status_reason.c_str());
                last_pending_log_wall_sec_ = wall_now_sec;
            }
            return;
        }
        pending_active_ = false;
        pending_since_wall_sec_ = 0.0;
        pending_error_reported_ = false;
        const bool urgent_warning =
            code == AlarmStateMachine::kLevel1Warning ||
            code == AlarmStateMachine::kLevel2Warning;
        const bool warning_repeat_due = urgent_warning &&
            (last_warning_log_wall_sec_ <= 0.0 ||
                 wall_now_sec - last_warning_log_wall_sec_ >=
                 warning_repeat_sec_);
        const bool periodic_fault =
            code == AlarmStateMachine::kCargoInvalid ||
            code == AlarmStateMachine::kObstacleInvalid;
        const bool fault_repeat_due = periodic_fault &&
            (last_fault_log_wall_sec_ <= 0.0 ||
             wall_now_sec - last_fault_log_wall_sec_ >=
                 pending_repeat_sec_);
        if (!log_change && !warning_repeat_due && !fault_repeat_due) return;
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
                     "cargo_track=%u obstacle_track=%u confidence=%.2f "
                     "reason=%s",
                     code, code == AlarmStateMachine::kLevel1Warning ? 1 : 2,
                     last_status_.nearest_obstacle_distance_m,
                     last_status_.conservative_vertical_clearance_m,
                     last_status_.cargo_bottom_z_map,
                     last_status_.obstacle_top_z_map,
                     last_status_.cargo_track_id,
                     last_status_.obstacle_track_id,
                     last_status_.confidence,
                     last_status_.reason.c_str());
            last_warning_log_wall_sec_ = wall_now_sec;
        } else {
            const bool message_reason = std::string(reason) == "fresh_status" ||
                std::string(reason) == "heartbeat";
            const std::string fault_reason = has_last_status_ && message_reason
                ? last_status_.reason : std::string(reason);
            if (code == AlarmStateMachine::kSystemNotReady) {
                ROS_WARN("[SAFETY_FAULT] code=%d reason=%s",
                         code, fault_reason.c_str());
            } else {
                ROS_ERROR("[SAFETY_FAULT%s] code=%d reason=%s",
                          !log_change && fault_repeat_due ? "_SUMMARY" : "",
                          code, fault_reason.c_str());
            }
            if (periodic_fault) last_fault_log_wall_sec_ = wall_now_sec;
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
    const double pending_error_sec_;
    const double pending_repeat_sec_;
    const double warning_repeat_sec_;
    const StatusContractConfig contract_config_;
    AlarmStateMachine state_machine_;
    lidar_slam2_msgs::CargoSafetyStatus last_status_;
    bool has_last_status_ = false;
    bool pending_active_ = false;
    bool pending_error_reported_ = false;
    double pending_since_wall_sec_ = 0.0;
    double last_pending_log_wall_sec_ = 0.0;
    double last_warning_log_wall_sec_ = 0.0;
    double last_fault_log_wall_sec_ = 0.0;
};

}  // namespace cargo_alarm

int main(int argc, char** argv) {
    ros::init(argc, argv, "cargo_alarm_heartbeat");
    cargo_alarm::AlarmHeartbeatNode node;
    ros::spin();
    return 0;
}

#endif  // CARGO_ALARM_HEARTBEAT_STATE_MACHINE_ONLY
