#pragma once

#include <optional>
#include <utility>

#include "ros_tracker/core/status.hpp"

namespace ros_tracker::core {

template <typename T>
class Result {
 public:
  Result(const T& value) : value_(value), status_(Status::ok_status()) {}
  Result(T&& value) : value_(std::move(value)), status_(Status::ok_status()) {}
  Result(const Status& status) : status_(status) {}
  Result(Status&& status) : status_(std::move(status)) {}

  [[nodiscard]] bool ok() const noexcept {
    return status_.ok();
  }

  [[nodiscard]] const Status& status() const noexcept {
    return status_;
  }

  [[nodiscard]] const T& value() const {
    return value_.value();
  }

  [[nodiscard]] T& value() {
    return value_.value();
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
  std::optional<T> value_;
  Status status_;
};

}  // namespace ros_tracker::core
