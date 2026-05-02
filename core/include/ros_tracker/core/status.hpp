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

  [[nodiscard]] bool ok() const noexcept {
    return code == StatusCode::kOk;
  }

  [[nodiscard]] static Status ok_status() {
    return {};
  }

  [[nodiscard]] static Status invalid_argument(std::string message) {
    return {StatusCode::kInvalidArgument, std::move(message)};
  }

  [[nodiscard]] static Status dimension_mismatch(std::string message) {
    return {StatusCode::kDimensionMismatch, std::move(message)};
  }

  [[nodiscard]] static Status out_of_range(std::string message) {
    return {StatusCode::kOutOfRange, std::move(message)};
  }

  [[nodiscard]] static Status numerical_error(std::string message) {
    return {StatusCode::kNumericalError, std::move(message)};
  }

  [[nodiscard]] static Status internal_error(std::string message) {
    return {StatusCode::kInternalError, std::move(message)};
  }

  [[nodiscard]] static Status unimplemented(std::string message) {
    return {StatusCode::kUnimplemented, std::move(message)};
  }
};

}  // namespace ros_tracker::core
