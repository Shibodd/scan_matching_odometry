#ifndef SCANMATCHINGODOMETRY_MATCHER_HPP
#define SCANMATCHINGODOMETRY_MATCHER_HPP

#include <pointmatcher/PointMatcher.h>
#include <optional>
#include <chrono>

#include <Eigen/Dense>

namespace scan_matching_odometry {
  enum class TwistTimestampMode {
    Previous,
    Average,
    Current
  };

  struct Pose {
    std::chrono::nanoseconds t;
    Eigen::Matrix3d transform;
  };

  struct Twist {
    std::chrono::nanoseconds t;
    Eigen::Vector2d linear;
    double angular;
  };

  struct Odometry {
    Pose pose;
    std::optional<Twist> twist;
  };

  class Matcher {
    struct OldData {
      PointMatcher<double>::DataPoints dp;
      std::chrono::nanoseconds t;
    };

    std::optional<OldData> m_old_data = std::nullopt;
    Eigen::Matrix<double, 3, 3> m_transform = decltype(m_transform)::Identity();

  public:
    Odometry update(std::chrono::nanoseconds t, const PointMatcher<double>::DataPoints& dp, TwistTimestampMode twist_timestamp_mode = TwistTimestampMode::Average);
  };
} // namespace scan_matching_odometry

#endif // !SCANMATCHINGODOMETRY_MATCHER_HPP