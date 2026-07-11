#include <cmath>
#include <cstdint>
#include <string>

namespace cargo_alarm {

class AlarmStateMachine {
public:
    static constexpr int kClear = 14;
    static constexpr int kInnerWarning = 17;
    static constexpr int kOuterOrInvalid = 18;

    struct Result {
        int code = kOuterOrInvalid;
        bool changed = false;
        const char* reason = "startup_fail_safe";
    };

    AlarmStateMachine(double stale_timeout_sec, double clear_delay_sec)
        : stale_timeout_sec_(sanitizePositive(stale_timeout_sec, 0.8)),
          clear_delay_sec_(sanitizeNonNegative(clear_delay_sec, 0.5)) {}

    int currentCode() const { return current_code_; }

    Result ingest(bool evidence_valid,
                  int requested_code,
                  double wall_now_sec,
                  double source_stamp_sec) {
        if (!observeWallClock(wall_now_sec)) {
            return forceFailSafe("wall_time_rollback");
        }

        has_status_ = true;
        last_receipt_wall_sec_ = wall_now_sec;

        if (!std::isfinite(source_stamp_sec) || source_stamp_sec <= 0.0) {
            return forceFailSafe("invalid_source_stamp");
        }
        if (has_source_stamp_ && source_stamp_sec + kTimeEpsilonSec < last_source_stamp_sec_) {
            // Establish a new epoch after the one-shot fail-safe so bag loops or
            // a restarted upstream clock can recover on the next advancing stamp.
            last_source_stamp_sec_ = source_stamp_sec;
            last_source_progress_wall_sec_ = wall_now_sec;
            return forceFailSafe("source_time_rollback");
        }
        if (!has_source_stamp_ ||
            source_stamp_sec > last_source_stamp_sec_ + kTimeEpsilonSec) {
            last_source_stamp_sec_ = source_stamp_sec;
            last_source_progress_wall_sec_ = wall_now_sec;
        }
        has_source_stamp_ = true;

        if (wall_now_sec - last_source_progress_wall_sec_ + kTimeEpsilonSec >=
            stale_timeout_sec_) {
            return forceFailSafe("source_stamp_stale");
        }

        if (!evidence_valid) {
            return forceFailSafe("invalid_evidence");
        }
        if (!isAllowedCode(requested_code)) {
            return forceFailSafe("invalid_alarm_code");
        }

        evidence_healthy_ = true;
        last_candidate_code_ = requested_code;
        return applyCandidate(requested_code, wall_now_sec, "fresh_status");
    }

    Result tick(double wall_now_sec) {
        if (!observeWallClock(wall_now_sec)) {
            return forceFailSafe("wall_time_rollback");
        }
        if (!has_status_ || !evidence_healthy_) {
            return forceFailSafe(has_status_ ? "invalid_evidence" : "startup_fail_safe");
        }
        if (wall_now_sec - last_receipt_wall_sec_ + kTimeEpsilonSec >= stale_timeout_sec_) {
            return forceFailSafe("status_stale");
        }
        if (wall_now_sec - last_source_progress_wall_sec_ + kTimeEpsilonSec >=
            stale_timeout_sec_) {
            return forceFailSafe("source_stamp_stale");
        }
        return applyCandidate(last_candidate_code_, wall_now_sec, "heartbeat");
    }

private:
    static constexpr double kTimeEpsilonSec = 1e-6;

    static double sanitizePositive(double value, double fallback) {
        return std::isfinite(value) && value > 0.0 ? value : fallback;
    }

    static double sanitizeNonNegative(double value, double fallback) {
        return std::isfinite(value) && value >= 0.0 ? value : fallback;
    }

    static bool isAllowedCode(int code) {
        return code == kClear || code == kInnerWarning || code == kOuterOrInvalid;
    }

    static int severity(int code) {
        if (code == kInnerWarning) {
            return 2;
        }
        if (code == kOuterOrInvalid) {
            return 1;
        }
        return 0;
    }

    bool observeWallClock(double wall_now_sec) {
        if (!std::isfinite(wall_now_sec) || wall_now_sec < 0.0) {
            return false;
        }
        if (has_wall_observation_ &&
            wall_now_sec + kTimeEpsilonSec < last_observed_wall_sec_) {
            last_observed_wall_sec_ = wall_now_sec;
            return false;
        }
        has_wall_observation_ = true;
        last_observed_wall_sec_ = wall_now_sec;
        return true;
    }

    Result forceFailSafe(const char* reason) {
        const bool changed = current_code_ != kOuterOrInvalid;
        current_code_ = kOuterOrInvalid;
        evidence_healthy_ = false;
        clear_pending_ = false;
        last_candidate_code_ = kOuterOrInvalid;
        return {current_code_, changed, reason};
    }

    Result applyCandidate(int candidate, double wall_now_sec, const char* reason) {
        if (severity(candidate) > severity(current_code_)) {
            const bool changed = candidate != current_code_;
            current_code_ = candidate;
            clear_pending_ = false;
            return {current_code_, changed, reason};
        }

        if (candidate == kClear && current_code_ != kClear) {
            if (!clear_pending_) {
                clear_pending_ = true;
                clear_pending_since_wall_sec_ = wall_now_sec;
                return {current_code_, false, "clear_delay_started"};
            }
            if (wall_now_sec - clear_pending_since_wall_sec_ + kTimeEpsilonSec >=
                clear_delay_sec_) {
                current_code_ = kClear;
                clear_pending_ = false;
                return {current_code_, true, "clear_delay_satisfied"};
            }
            return {current_code_, false, "clear_delay_pending"};
        }

        clear_pending_ = false;
        if (candidate != current_code_) {
            current_code_ = candidate;
            return {current_code_, true, reason};
        }
        return {current_code_, false, reason};
    }

    const double stale_timeout_sec_;
    const double clear_delay_sec_;
    int current_code_ = kOuterOrInvalid;
    int last_candidate_code_ = kOuterOrInvalid;
    bool has_status_ = false;
    bool evidence_healthy_ = false;
    bool has_wall_observation_ = false;
    bool has_source_stamp_ = false;
    bool clear_pending_ = false;
    double last_receipt_wall_sec_ = 0.0;
    double last_observed_wall_sec_ = 0.0;
    double last_source_stamp_sec_ = 0.0;
    double last_source_progress_wall_sec_ = 0.0;
    double clear_pending_since_wall_sec_ = 0.0;
};

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
          clear_delay_sec_(readNonNegativeParam("clear_delay_sec", 0.5)),
          state_machine_(stale_timeout_sec_, clear_delay_sec_) {
        pnh_.param<std::string>("status_topic", status_topic_,
                                "/cargo_avoidance/safety_status");
        pnh_.param<std::string>("alarm_topic", alarm_topic_,
                                "/cargo_avoidance/alarm_code");

        int status_queue_size = 1;
        pnh_.param("status_queue_size", status_queue_size, status_queue_size);
        if (status_queue_size != 1) {
            ROS_WARN("[CargoAlarmHeartbeat] status_queue_size=%d is unsafe; using 1",
                     status_queue_size);
            status_queue_size = 1;
        }

        alarm_pub_ = nh_.advertise<std_msgs::Int32>(alarm_topic_, 1, true);
        status_sub_ = nh_.subscribe(status_topic_, status_queue_size,
                                    &AlarmHeartbeatNode::statusCallback, this);

        // The latched topic is fail-safe from the moment this node starts.
        publishAlarm(AlarmStateMachine::kOuterOrInvalid, "startup_fail_safe");
        heartbeat_timer_ = nh_.createWallTimer(
            ros::WallDuration(1.0 / heartbeat_hz_),
            &AlarmHeartbeatNode::heartbeatCallback, this);

        ROS_INFO("[CargoAlarmHeartbeat] status=%s alarm=%s heartbeat=%.2fHz "
                 "stale=%.3fs clear_delay=%.3fs",
                 status_topic_.c_str(), alarm_topic_.c_str(), heartbeat_hz_,
                 stale_timeout_sec_, clear_delay_sec_);
    }

private:
    double readPositiveParam(const std::string& name, double fallback) {
        double value = fallback;
        pnh_.param(name, value, fallback);
        if (!std::isfinite(value) || value <= 0.0) {
            ROS_WARN("[CargoAlarmHeartbeat] ~%s=%.6f is invalid; using %.6f",
                     name.c_str(), value, fallback);
            return fallback;
        }
        return value;
    }

    double readNonNegativeParam(const std::string& name, double fallback) {
        double value = fallback;
        pnh_.param(name, value, fallback);
        if (!std::isfinite(value) || value < 0.0) {
            ROS_WARN("[CargoAlarmHeartbeat] ~%s=%.6f is invalid; using %.6f",
                     name.c_str(), value, fallback);
            return fallback;
        }
        return value;
    }

    void statusCallback(const lidar_slam2_msgs::CargoSafetyStatus::ConstPtr& msg) {
        const bool schema_valid =
            msg->schema_version == lidar_slam2_msgs::CargoSafetyStatus::SCHEMA_VERSION;
        const bool evidence_valid = msg->valid && msg->cargo_valid && schema_valid;
        const AlarmStateMachine::Result result = state_machine_.ingest(
            evidence_valid, msg->requested_alarm_code,
            ros::WallTime::now().toSec(), msg->header.stamp.toSec());

        // Escalation and invalid evidence are published in this callback, without
        // waiting for the periodic heartbeat.
        if (result.changed || result.code == AlarmStateMachine::kOuterOrInvalid) {
            publishAlarm(result.code, result.reason);
        }
    }

    void heartbeatCallback(const ros::WallTimerEvent& event) {
        const AlarmStateMachine::Result result =
            state_machine_.tick(event.current_real.toSec());
        publishAlarm(result.code, result.reason);
    }

