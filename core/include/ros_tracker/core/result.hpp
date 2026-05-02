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
  Result(const T& value) : value_(value), status_(Status::ok_status()) {}
  Result(T&& value) : value_(std::move(value)), status_(Status::ok_status()) {}
  Result(const Status& status) : status_(normalize_status(status)) {}
  Result(Status&& status) : status_(normalize_status(std::move(status))) {}

  [[nodiscard]] bool ok() const noexcept {
    return status_.ok();
  }

  [[nodiscard]] explicit operator bool() const noexcept {
    return ok();
  }

  [[nodiscard]] bool has_value() const noexcept {
    return value_.has_value();
  }

  [[nodiscard]] const Status& status() const noexcept {
    return status_;
  }

  [[nodiscard]] const T& value() const {
    ensure_value();
    return *value_;
  }

  [[nodiscard]] T& value() {
    ensure_value();
    return *value_;
  }

  [[nodiscard]] const T& operator*() const {
    return value();
  }

  [[nodiscard]] T& operator*() {
    return value();
  }

  [[nodiscard]] const T* operator->() const {
    return &value();
  }

  [[nodiscard]] T* operator->() {
    return &value();
  }

 private:
  [[nodiscard]] static Status normalize_status(Status status) {
    if (status.ok()) {
      return Status::internal_error(
          "Result<T> cannot be constructed from an OK status without a value.");
    }

    return status;
  }

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
