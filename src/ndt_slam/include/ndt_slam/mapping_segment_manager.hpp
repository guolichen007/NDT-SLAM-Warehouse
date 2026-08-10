#pragma once

#include <cstdint>
#include <mutex>
#include <string>

namespace ndt_slam {

enum class MappingSegmentState {
    WAIT_OPERATOR,
    RUNNING,
    CLOSED,
    FAILED_CLOSED,
    ABORTED_CRASH,
    ARCHIVE_INCOMPLETE
};

const char* mappingSegmentStateName(MappingSegmentState state);

struct MappingSegmentConfig {
    bool enabled = false;
    std::string root_dir;
    std::string campaign_uuid;
    std::string survey_pass_id;
};

struct MappingSegmentSnapshot {
    MappingSegmentState state = MappingSegmentState::WAIT_OPERATOR;
    std::string campaign_uuid;
    std::string survey_pass_id;
    std::string segment_uuid;
    std::string reason = "not_initialized";
    std::uint64_t sequence = 0U;
    double transition_wall_sec = 0.0;
    bool active = false;
    bool writes_allowed = false;
    bool previous_crash_detected = false;
};

struct MappingSegmentStartPrerequisites {
    bool source_time_continuous = false;
    bool self_mask_commissioned = false;
    bool archive_healthy = false;
    bool archive_idle = false;
};

// Segment filesystem operations are used only during startup and the explicit
// operator service. Runtime failure transitions update memory immediately and
// provide stateJson() for the asynchronous archive worker.
class MappingSegmentManager {
public:
    void configure(const MappingSegmentConfig& config);
    bool initialize(std::string* reason);
    bool startNewSegment(
        const MappingSegmentStartPrerequisites& prerequisites,
        std::string* reason);
    void failClosed(const std::string& reason);
    void markArchiveIncomplete(const std::string& reason);
    void close(const std::string& reason);

    MappingSegmentSnapshot snapshot() const;
    std::string stateJson() const;
    std::string stateRelativePath() const;
    bool enabled() const;

private:
    bool persistStateAndRunningLock(std::string* reason);
    bool sealPreviousRunningLock(std::string* reason);
    std::string createSegmentUuid() const;

    MappingSegmentConfig config_;
    mutable std::mutex mutex_;
    MappingSegmentSnapshot snapshot_;
};

}  // namespace ndt_slam
