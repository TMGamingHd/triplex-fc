// SPDX-License-Identifier: MIT
// Frame progress monitor (TFC-FDIR-038). The hardware watchdog must be serviced from exactly one place, at the end of a
// completed frame, and only if the vote ran and every monitored task has reported progress in that frame. A task that
// stops reporting then stops the servicing, whatever the other tasks keep doing (the Mars rover's Sol 200 anomaly:
// enough live tasks kept feeding the watchdog while others were hung).
// No heap, no exceptions, no RTTI. Deterministic.
#pragma once
#include <cstdint>

namespace tfc {

constexpr unsigned kMaxProgressTasks = 8U;

class ProgressMonitor {
 public:
  // `required`: bit i set means task i must report in every frame.
  explicit constexpr ProgressMonitor(uint8_t required) noexcept : required_(required) {}

  // A monitored task reports that it made progress in the current frame. A task number outside 0..7 is counted and ignored.
  void report(unsigned task) noexcept {
    if (task >= kMaxProgressTasks) {
      ++bad_reports_;
      return;
    }
    seen_ = static_cast<uint8_t>(seen_ | (1U << task));
  }

  // The frame is over. True only if the vote ran and every required task reported; the report word is cleared either way,
  // so a task that reported in an earlier frame never covers for itself.
  [[nodiscard]] bool end_of_frame(bool vote_ran) noexcept {
    missing_ = static_cast<uint8_t>(required_ & static_cast<uint8_t>(~seen_));
    seen_ = 0U;
    const bool ok = vote_ran && missing_ == 0U;
    if (!ok) {
      ++refusals_;
    }
    return ok;
  }

  // Required tasks that did not report in the frame that just ended.
  [[nodiscard]] uint8_t missing() const noexcept { return missing_; }
  // Frames in which servicing was refused.
  [[nodiscard]] uint32_t refusals() const noexcept { return refusals_; }
  // Reports from a task number that does not exist.
  [[nodiscard]] uint32_t bad_reports() const noexcept { return bad_reports_; }

 private:
  uint8_t required_;
  uint8_t seen_ = 0U;
  uint8_t missing_ = 0U;
  uint32_t refusals_ = 0U;
  uint32_t bad_reports_ = 0U;
};

}  // namespace tfc
