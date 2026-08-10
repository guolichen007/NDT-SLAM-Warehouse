#include <ndt_slam/mapping_segment_manager.hpp>
#include <ndt_slam/sha256.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <random>
#include <sstream>
#include <system_error>
#include <utility>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace ndt_slam {
namespace fs = std::filesystem;

namespace {

double wallSeconds() {
    return std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string escapeJson(const std::string& value) {
    std::ostringstream output;
    for (const char character : value) {
        switch (character) {
        case '\\': output << "\\\\"; break;
        case '"': output << "\\\""; break;
        case '\n': output << "\\n"; break;
        case '\r': output << "\\r"; break;
        case '\t': output << "\\t"; break;
        default: output << character; break;
        }
    }
    return output.str();
}

std::string snapshotJson(const MappingSegmentSnapshot& snapshot) {
    std::ostringstream output;
    output << std::fixed << std::setprecision(6)
           << "{\n"
           << "  \"schema_version\": 1,\n"
           << "  \"campaign_uuid\": \""
           << escapeJson(snapshot.campaign_uuid) << "\",\n"
           << "  \"survey_pass_id\": \""
           << escapeJson(snapshot.survey_pass_id) << "\",\n"
           << "  \"segment_uuid\": \""
           << escapeJson(snapshot.segment_uuid) << "\",\n"
           << "  \"state\": \"" << mappingSegmentStateName(snapshot.state)
           << "\",\n"
           << "  \"reason\": \"" << escapeJson(snapshot.reason) << "\",\n"
           << "  \"sequence\": " << snapshot.sequence << ",\n"
           << "  \"transition_wall_sec\": "
           << snapshot.transition_wall_sec << "\n"
           << "}\n";
    return output.str();
}

bool syncFile(const fs::path& path) {
#ifdef _WIN32
    const int descriptor = _open(
        path.string().c_str(), _O_RDWR | _O_BINARY);
    if (descriptor < 0) return false;
    const bool ok = _commit(descriptor) == 0;
    _close(descriptor);
    return ok;
#else
    const int descriptor = ::open(
        path.c_str(), O_RDONLY | O_CLOEXEC);
    if (descriptor < 0) return false;
    const bool ok = ::fsync(descriptor) == 0;
    ::close(descriptor);
    return ok;
#endif
}

bool syncDirectory(const fs::path& path) {
#ifdef _WIN32
    (void)path;
    return true;
#else
    const int descriptor = ::open(
        path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (descriptor < 0) return false;
    const bool ok = ::fsync(descriptor) == 0;
    ::close(descriptor);
    return ok;
#endif
}

bool atomicWrite(const fs::path& target, const std::string& contents) {
    std::error_code error;
    fs::create_directories(target.parent_path(), error);
    if (error) return false;
    const fs::path temporary = target.string() + ".tmp";
    {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        output.write(contents.data(),
                     static_cast<std::streamsize>(contents.size()));
        output.flush();
        if (!output.good()) return false;
    }
    if (!syncFile(temporary)) {
        fs::remove(temporary, error);
        return false;
    }
#ifdef _WIN32
    // std::filesystem::rename cannot replace an existing file on Windows.
    // Production runs on Linux, where the single rename below atomically
    // replaces the prior state file.
    fs::remove(target, error);
    error.clear();
#endif
    fs::rename(temporary, target, error);
    return !error && syncDirectory(target.parent_path());
}

bool readJsonStringField(const std::string& contents,
                         const std::string& key,
                         std::string* value);

bool checksumSidecarValid(const fs::path& target) {
    std::ifstream sidecar(target.string() + ".sha256", std::ios::binary);
    std::string expected;
    sidecar >> expected;
    return expected.size() == 64U &&
        sha256File(target.string()) == expected;
}

bool readTerminalArchivedState(const fs::path& root,
                               const std::string& segment_uuid,
                               MappingSegmentState* state) {
    if (!state || segment_uuid.empty()) return false;
    const fs::path archived = root / "segments" / segment_uuid /
        "state.json";
    if (!checksumSidecarValid(archived)) return false;
    std::ifstream input(archived, std::ios::binary);
    if (!input.is_open()) return false;
    const std::string contents(
        (std::istreambuf_iterator<char>(input)),
        std::istreambuf_iterator<char>());
    std::string archived_segment_uuid;
    if (!readJsonStringField(contents, "segment_uuid",
                             &archived_segment_uuid) ||
        archived_segment_uuid != segment_uuid) {
        return false;
    }
    const struct {
        const char* token;
        MappingSegmentState state;
    } terminal_states[] = {
        {"\"state\": \"FAILED_CLOSED\"",
         MappingSegmentState::FAILED_CLOSED},
        {"\"state\": \"ARCHIVE_INCOMPLETE\"",
         MappingSegmentState::ARCHIVE_INCOMPLETE},
        {"\"state\": \"CLOSED\"", MappingSegmentState::CLOSED},
    };
    for (const auto& candidate : terminal_states) {
        if (contents.find(candidate.token) != std::string::npos) {
            *state = candidate.state;
            return true;
        }
    }
    return false;
}

bool readJsonStringField(const std::string& contents,
                         const std::string& key,
                         std::string* value) {
    if (!value) return false;
    const std::string token = "\"" + key + "\"";
    std::size_t position = contents.find(token);
    if (position == std::string::npos) return false;
    position = contents.find(':', position + token.size());
    if (position == std::string::npos) return false;
    position = contents.find('"', position + 1U);
    if (position == std::string::npos) return false;
    ++position;
    std::string parsed;
    bool escaped = false;
    for (; position < contents.size(); ++position) {
        const char character = contents[position];
        if (escaped) {
            switch (character) {
            case 'n': parsed.push_back('\n'); break;
            case 'r': parsed.push_back('\r'); break;
            case 't': parsed.push_back('\t'); break;
            default: parsed.push_back(character); break;
            }
            escaped = false;
        } else if (character == '\\') {
            escaped = true;
        } else if (character == '"') {
            *value = std::move(parsed);
            return true;
        } else {
            parsed.push_back(character);
        }
    }
    return false;
}

bool parseMappingSegmentState(const std::string& value,
                              MappingSegmentState* state) {
    if (!state) return false;
    const struct {
        const char* name;
        MappingSegmentState state;
    } candidates[] = {
        {"WAIT_OPERATOR", MappingSegmentState::WAIT_OPERATOR},
        {"RUNNING", MappingSegmentState::RUNNING},
        {"CLOSED", MappingSegmentState::CLOSED},
        {"FAILED_CLOSED", MappingSegmentState::FAILED_CLOSED},
        {"ABORTED_CRASH", MappingSegmentState::ABORTED_CRASH},
        {"ARCHIVE_INCOMPLETE", MappingSegmentState::ARCHIVE_INCOMPLETE},
    };
    for (const auto& candidate : candidates) {
        if (value == candidate.name) {
            *state = candidate.state;
            return true;
        }
    }
    return false;
}

bool readPersistedRootState(const fs::path& root,
                            MappingSegmentSnapshot* snapshot) {
    if (!snapshot) return false;
    std::ifstream input(root / "state.json", std::ios::binary);
    if (!input.is_open()) return false;
    const std::string contents(
        (std::istreambuf_iterator<char>(input)),
        std::istreambuf_iterator<char>());
    std::string state_name;
    MappingSegmentSnapshot restored = *snapshot;
    if (!readJsonStringField(contents, "campaign_uuid",
                             &restored.campaign_uuid) ||
        !readJsonStringField(contents, "survey_pass_id",
                             &restored.survey_pass_id) ||
        !readJsonStringField(contents, "segment_uuid",
                             &restored.segment_uuid) ||
        !readJsonStringField(contents, "state", &state_name) ||
        !readJsonStringField(contents, "reason", &restored.reason) ||
        !parseMappingSegmentState(state_name, &restored.state)) {
        return false;
    }
    *snapshot = std::move(restored);
    return true;
}

}  // namespace

const char* mappingSegmentStateName(MappingSegmentState state) {
    switch (state) {
    case MappingSegmentState::WAIT_OPERATOR: return "WAIT_OPERATOR";
    case MappingSegmentState::RUNNING: return "RUNNING";
    case MappingSegmentState::CLOSED: return "CLOSED";
    case MappingSegmentState::FAILED_CLOSED: return "FAILED_CLOSED";
    case MappingSegmentState::ABORTED_CRASH: return "ABORTED_CRASH";
    case MappingSegmentState::ARCHIVE_INCOMPLETE:
        return "ARCHIVE_INCOMPLETE";
    }
    return "UNKNOWN";
}

void MappingSegmentManager::configure(const MappingSegmentConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
    snapshot_ = {};
    snapshot_.campaign_uuid = config_.campaign_uuid;
    snapshot_.survey_pass_id = config_.survey_pass_id;
    snapshot_.reason = config_.enabled ? "wait_operator" : "disabled";
}

bool MappingSegmentManager::initialize(std::string* reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!config_.enabled) {
        snapshot_.reason = "disabled";
        if (reason) *reason = snapshot_.reason;
        return true;
    }
    if (config_.root_dir.empty() || config_.campaign_uuid.empty() ||
        config_.survey_pass_id.empty()) {
        snapshot_.reason = "campaign_or_survey_pass_not_configured";
        if (reason) *reason = snapshot_.reason;
        return false;
    }
    std::error_code error;
    fs::create_directories(config_.root_dir, error);
    if (error) {
        snapshot_.reason = "segment_root_create_failed";
        if (reason) *reason = snapshot_.reason;
        return false;
    }
    const fs::path running = fs::path(config_.root_dir) / "RUNNING.lock";
    const bool running_exists = fs::exists(running, error);
    if (error) {
        snapshot_.state = MappingSegmentState::ABORTED_CRASH;
        snapshot_.previous_crash_detected = true;
        snapshot_.reason = "running_lock_stat_failed";
        snapshot_.active = false;
        snapshot_.writes_allowed = false;
        if (reason) *reason = snapshot_.reason;
        return false;
    }
    bool initialization_ok = true;
    if (running_exists) {
        std::ifstream input(running);
        std::getline(input, snapshot_.segment_uuid);
        MappingSegmentState archived_state;
        const bool terminal_state_archived = readTerminalArchivedState(
            fs::path(config_.root_dir), snapshot_.segment_uuid,
            &archived_state);
        snapshot_.state = terminal_state_archived
            ? archived_state : MappingSegmentState::ABORTED_CRASH;
        snapshot_.previous_crash_detected = !terminal_state_archived;
        snapshot_.reason = terminal_state_archived
            ? "startup_recovered_archived_terminal_state"
            : "startup_found_running_lock";
        snapshot_.transition_wall_sec = wallSeconds();
        ++snapshot_.sequence;
        const fs::path aborted = fs::path(config_.root_dir) /
            (std::string(mappingSegmentStateName(snapshot_.state)) + "_" +
             (snapshot_.segment_uuid.empty() ? "unknown" :
              snapshot_.segment_uuid) + ".lock");
        fs::rename(running, aborted, error);
        if (error) {
            snapshot_.reason = "running_lock_seal_failed";
            initialization_ok = false;
        }
        if (!atomicWrite(fs::path(config_.root_dir) / "state.json",
                         snapshotJson(snapshot_))) {
            snapshot_.reason = "aborted_crash_state_write_failed";
            initialization_ok = false;
        }
    } else {
        MappingSegmentSnapshot persisted = snapshot_;
        if (readPersistedRootState(fs::path(config_.root_dir), &persisted)) {
            const bool campaign_identity_valid =
                persisted.campaign_uuid == config_.campaign_uuid &&
                persisted.survey_pass_id == config_.survey_pass_id;
            snapshot_ = std::move(persisted);
            if (!campaign_identity_valid ||
                snapshot_.state == MappingSegmentState::RUNNING) {
                snapshot_.state = MappingSegmentState::ABORTED_CRASH;
                snapshot_.previous_crash_detected = true;
                snapshot_.reason = !campaign_identity_valid
                    ? "persisted_campaign_identity_mismatch"
                    : "persisted_running_state_without_lock";
                ++snapshot_.sequence;
                if (!atomicWrite(fs::path(config_.root_dir) / "state.json",
                                 snapshotJson(snapshot_))) {
                    snapshot_.reason =
                        "persisted_abort_state_write_failed";
                    initialization_ok = false;
                }
            } else {
                snapshot_.previous_crash_detected =
                    snapshot_.state == MappingSegmentState::ABORTED_CRASH;
                snapshot_.reason = "startup_restored_persisted_" +
                    std::string(mappingSegmentStateName(snapshot_.state));
            }
            snapshot_.transition_wall_sec = wallSeconds();
        } else {
            snapshot_.state = MappingSegmentState::WAIT_OPERATOR;
            snapshot_.reason = "operator_start_required";
            snapshot_.transition_wall_sec = wallSeconds();
        }
    }
    snapshot_.active = false;
    snapshot_.writes_allowed = false;
    if (reason) *reason = snapshot_.reason;
    return initialization_ok;
}

std::string MappingSegmentManager::createSegmentUuid() const {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
        now).count();
    std::random_device random;
    std::ostringstream output;
    output << config_.campaign_uuid << '-' << config_.survey_pass_id << '-'
           << std::hex << micros << '-' << random();
    return output.str();
}

bool MappingSegmentManager::sealPreviousRunningLock(std::string* reason) {
    const fs::path running = fs::path(config_.root_dir) / "RUNNING.lock";
    std::error_code error;
    const bool exists = fs::exists(running, error);
    if (error) {
        if (reason) *reason = "previous_running_lock_stat_failed";
        return false;
    }
    if (!exists) return true;
    const fs::path sealed = fs::path(config_.root_dir) /
        (std::string(mappingSegmentStateName(snapshot_.state)) + '_' +
         (snapshot_.segment_uuid.empty() ? "unknown" :
          snapshot_.segment_uuid) + ".lock");
    fs::rename(running, sealed, error);
    if (error) {
        if (reason) *reason = "previous_running_lock_seal_failed";
        return false;
    }
    return true;
}

bool MappingSegmentManager::persistStateAndRunningLock(std::string* reason) {
    const fs::path root(config_.root_dir);
    if (!atomicWrite(root / "state.json", snapshotJson(snapshot_)) ||
        !atomicWrite(root / "RUNNING.lock", snapshot_.segment_uuid + "\n")) {
        if (reason) *reason = "segment_state_write_failed";
        return false;
    }
    return true;
}

bool MappingSegmentManager::startNewSegment(
    const MappingSegmentStartPrerequisites& prerequisites,
    std::string* reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!config_.enabled) {
        if (reason) *reason = "segment_manager_disabled";
        return false;
    }
    if (snapshot_.active) {
        if (reason) *reason = "segment_already_running";
        return false;
    }
    if (!prerequisites.source_time_continuous ||
        !prerequisites.self_mask_commissioned ||
        !prerequisites.archive_healthy || !prerequisites.archive_idle) {
        if (reason) {
            *reason = !prerequisites.source_time_continuous
                ? "pointcloud_source_time_not_continuous"
                : (!prerequisites.self_mask_commissioned
                    ? "self_mask_not_commissioned"
                    : (!prerequisites.archive_healthy
                        ? "archive_not_healthy" : "archive_not_idle"));
        }
        return false;
    }
    if (!sealPreviousRunningLock(reason)) return false;
    snapshot_.state = MappingSegmentState::RUNNING;
    snapshot_.segment_uuid = createSegmentUuid();
    snapshot_.reason = "operator_confirmed_stopped";
    snapshot_.transition_wall_sec = wallSeconds();
    snapshot_.active = true;
    snapshot_.writes_allowed = true;
    snapshot_.previous_crash_detected = false;
    ++snapshot_.sequence;
    if (!persistStateAndRunningLock(reason)) {
        snapshot_.state = MappingSegmentState::ARCHIVE_INCOMPLETE;
        snapshot_.reason = "segment_start_persistence_failed";
        snapshot_.active = false;
        snapshot_.writes_allowed = false;
        return false;
    }
    if (reason) *reason = snapshot_.reason;
    return true;
}

void MappingSegmentManager::failClosed(const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!snapshot_.active) return;
    snapshot_.state = MappingSegmentState::FAILED_CLOSED;
    snapshot_.reason = reason;
    snapshot_.transition_wall_sec = wallSeconds();
    snapshot_.active = false;
    snapshot_.writes_allowed = false;
    ++snapshot_.sequence;
}

void MappingSegmentManager::markArchiveIncomplete(
    const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.state = MappingSegmentState::ARCHIVE_INCOMPLETE;
    snapshot_.reason = reason;
    snapshot_.transition_wall_sec = wallSeconds();
    snapshot_.active = false;
    snapshot_.writes_allowed = false;
    ++snapshot_.sequence;
}

void MappingSegmentManager::close(const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.state = MappingSegmentState::CLOSED;
    snapshot_.reason = reason;
    snapshot_.transition_wall_sec = wallSeconds();
    snapshot_.active = false;
    snapshot_.writes_allowed = false;
    ++snapshot_.sequence;
}

MappingSegmentSnapshot MappingSegmentManager::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
}

std::string MappingSegmentManager::stateJson() const {
    return snapshotJson(snapshot());
}

std::string MappingSegmentManager::stateRelativePath() const {
    const MappingSegmentSnapshot value = snapshot();
    return value.segment_uuid.empty()
        ? "state.json"
        : "segments/" + value.segment_uuid + "/state.json";
}

bool MappingSegmentManager::enabled() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_.enabled;
}

}  // namespace ndt_slam
