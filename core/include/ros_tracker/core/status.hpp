#pragma once

#include <string>
#include <utility>

namespace ros_tracker::core {

enum class StatusCode {
  kOk = 0,
  kInvalidArgument,
  kDimensionMismatch,
  kOutOfRange,
  kNumericalError,
  kInternalError,
  kUnimplemented,
};

struct Status {
  StatusCode code {StatusCode::kOk};
  std::string message;

  /// Returns whether the operation succeeded.
  [[nodiscard]] bool ok() const noexcept {
    return code == StatusCode::kOk;
  }

  /// Creates an OK status.
  [[nodiscard]] static Status ok_status() {
    return {};
  }

  /// Creates an invalid-argument status.
  [[nodiscard]] static Status invalid_argument(std::string message) {
    return {StatusCode::kInvalidArgument, std::move(message)};
  }

  /// Creates a dimension-mismatch status.
  [[nodiscard]] static Status dimension_mismatch(std::string message) {
    return {StatusCode::kDimensionMismatch, std::move(message)};
  }

  /// Creates an out-of-range status.
  [[nodiscard]] static Status out_of_range(std::string message) {
    return {StatusCode::kOutOfRange, std::move(message)};
  }

  /// Creates a numerical-error status.
  [[nodiscard]] static Status numerical_error(std::string message) {
    return {StatusCode::kNumericalError, std::move(message)};
  }

  /// Creates an internal-error status.
  [[nodiscard]] static Status internal_error(std::string message) {
    return {StatusCode::kInternalError, std::move(message)};
  }

  /// Creates an unimplemented status.
  [[nodiscard]] static Status unimplemented(std::string message) {
    return {StatusCode::kUnimplemented, std::move(message)};
  }
};

}  // namespace ros_tracker::core
