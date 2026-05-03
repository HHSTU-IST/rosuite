#pragma once

#include <stdexcept>
#include <optional>
#include <string>
#include <utility>

#include "ros_tracker/core/status.hpp"

namespace ros_tracker::core {

template <typename T>
class Result {
 public:
  /// Constructs Result.
  Result(const T& value) : value_(value), status_(Status::ok_status()) {}
  /// Constructs Result.
  Result(T&& value) : value_(std::move(value)), status_(Status::ok_status()) {}
  /// Constructs Result.
  Result(const Status& status) : status_(normalize_status(status)) {}
  /// Constructs Result.
  Result(Status&& status) : status_(normalize_status(std::move(status))) {}

  /// Returns whether the operation succeeded.
  [[nodiscard]] bool ok() const noexcept {
    return status_.ok();
  }

  /// Returns whether the object represents success.
  [[nodiscard]] explicit operator bool() const noexcept {
    return ok();
  }

  /// Returns whether a value is present.
  [[nodiscard]] bool has_value() const noexcept {
    return value_.has_value();
  }

  /// Returns the current status.
  [[nodiscard]] const Status& status() const noexcept {
    return status_;
  }

  /// Returns the stored value.
  [[nodiscard]] const T& value() const {
    ensure_value();
    return *value_;
  }

  /// Returns the stored value.
  [[nodiscard]] T& value() {
    ensure_value();
    return *value_;
  }

  /// Returns the stored value by reference.
  [[nodiscard]] const T& operator*() const {
    return value();
  }

  /// Returns the stored value by reference.
  [[nodiscard]] T& operator*() {
    return value();
  }

  /// Returns a pointer to the stored value.
  [[nodiscard]] const T* operator->() const {
    return &value();
  }

  /// Returns a pointer to the stored value.
  [[nodiscard]] T* operator->() {
    return &value();
  }

 private:
  /// Normalizes invalid success statuses into internal errors.
  [[nodiscard]] static Status normalize_status(Status status) {
    if (status.ok()) {
      return Status::internal_error(
          "Result<T> cannot be constructed from an OK status without a value.");
    }

    return status;
  }

  /// Ensures that a value is available before access.
  void ensure_value() const {
    if (value_.has_value()) {
      return;
    }

    if (status_.ok()) {
      throw std::logic_error(
          "Result<T>::value() called on a successful result without a value.");
    }

    throw std::logic_error(
        "Result<T>::value() called on an error result: " + status_.message);
  }

  std::optional<T> value_;
  Status status_;
};

}  // namespace ros_tracker::core
