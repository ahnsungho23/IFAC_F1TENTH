// Copyright 2026 2026_IFAC contributors

#ifndef GLOBAL_PLANNING__FRENET_LAP_COUNTER_HPP_
#define GLOBAL_PLANNING__FRENET_LAP_COUNTER_HPP_

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace global_planning
{

class FrenetLapCounter
{
public:
  FrenetLapCounter(
    const double finish_s_min,
    const double start_s_max,
    const double min_lap_time_sec,
    const std::int32_t initial_lap_count = 0)
  : finish_s_min_(finish_s_min),
    start_s_max_(start_s_max),
    min_lap_time_sec_(min_lap_time_sec),
    lap_count_(initial_lap_count)
  {
    if (!std::isfinite(finish_s_min_) || !std::isfinite(start_s_max_) ||
      finish_s_min_ <= start_s_max_)
    {
      throw std::invalid_argument("finish_s_min must be finite and greater than start_s_max");
    }
    if (!std::isfinite(min_lap_time_sec_) || min_lap_time_sec_ < 0.0) {
      throw std::invalid_argument("min_lap_time_sec must be finite and non-negative");
    }
    if (lap_count_ < 0) {
      throw std::invalid_argument("initial_lap_count must be non-negative");
    }
  }

  bool update(const double s, const double sample_time_sec)
  {
    if (!std::isfinite(s) || !std::isfinite(sample_time_sec)) {
      return false;
    }

    if (!initialized_ || sample_time_sec < previous_sample_time_sec_) {
      initialized_ = true;
      previous_s_ = s;
      previous_sample_time_sec_ = sample_time_sec;
      last_wrap_time_sec_ = sample_time_sec;
      return false;
    }

    const bool crossed_wrap = previous_s_ >= finish_s_min_ && s <= start_s_max_;
    previous_s_ = s;
    previous_sample_time_sec_ = sample_time_sec;

    if (!crossed_wrap) {
      return false;
    }

    const double elapsed_sec = sample_time_sec - last_wrap_time_sec_;
    last_wrap_time_sec_ = sample_time_sec;
    if (elapsed_sec < min_lap_time_sec_) {
      return false;
    }

    if (lap_count_ == std::numeric_limits<std::int32_t>::max()) {
      return false;
    }

    ++lap_count_;
    return true;
  }

  std::int32_t lapCount() const
  {
    return lap_count_;
  }

private:
  double finish_s_min_;
  double start_s_max_;
  double min_lap_time_sec_;
  std::int32_t lap_count_;
  bool initialized_{false};
  double previous_s_{0.0};
  double previous_sample_time_sec_{0.0};
  double last_wrap_time_sec_{0.0};
};

}  // namespace global_planning

#endif  // GLOBAL_PLANNING__FRENET_LAP_COUNTER_HPP_
