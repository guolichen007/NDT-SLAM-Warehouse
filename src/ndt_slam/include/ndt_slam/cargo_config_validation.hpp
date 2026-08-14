#pragma once

#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace ndt_slam {

struct CargoConfigValidationIssue {
  std::string field;
  std::string reason;
};

struct CargoConfigValidationResult {
  bool valid = true;
  std::vector<CargoConfigValidationIssue> issues;

  void reject(std::string field, std::string reason) {
    valid = false;
    issues.push_back({std::move(field), std::move(reason)});
  }

  void append(const CargoConfigValidationResult& other,
              const std::string& prefix = {}) {
    if (other.valid) return;
    valid = false;
    for (const CargoConfigValidationIssue& issue : other.issues) {
      issues.push_back({prefix + issue.field, issue.reason});
    }
  }

  std::string summary() const {
    if (valid) return "valid";
    std::ostringstream stream;
    for (std::size_t i = 0U; i < issues.size(); ++i) {
      if (i != 0U) stream << ';';
      stream << issues[i].field << ':' << issues[i].reason;
    }
    return stream.str();
  }
};

}  // namespace ndt_slam
