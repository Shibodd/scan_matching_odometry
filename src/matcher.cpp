#include <scan_matching/matcher.hpp>

namespace scan_matching_odometry {


static inline std::chrono::nanoseconds get_twist_t(std::chrono::nanoseconds t, std::chrono::nanoseconds old_t, TwistTimestampMode mode) {
  switch (mode) {
    case TwistTimestampMode::Previous:
      return old_t;
      break;
    case TwistTimestampMode::Average:
      return old_t + (t - old_t) / 2;
      break;
    case TwistTimestampMode::Current:
      return t;
      break;
  }

  assert(false);
  return std::chrono::nanoseconds(0);
}


Odometry Matcher::update(std::chrono::nanoseconds t, const PointMatcher<double>::DataPoints& dp, TwistTimestampMode twist_timestamp_mode) {
  Odometry ans;

  if (m_old_data.has_value()) {
    PointMatcher<double>::ICP icp;
    icp.setDefault();
    PointMatcher<double>::TransformationParameters T = icp(dp, m_old_data->dp);
    assert(T.rows() == 3 && T.cols() == 3);

    Eigen::Ref<Eigen::Matrix3d> Tf(T);
    
    // Assert that the result is a rigid transformation
    assert((Tf(0,0) == Tf(1,1)) && (Tf(1,0) == -Tf(0,1)));
    assert(std::abs(Tf.topLeftCorner<2,2>().determinant() - 1) < 1e-8);
    assert((Tf.bottomLeftCorner<1,3>() == Eigen::Matrix<double, 1, 3>(0, 0, 1)));
  
    ans.pose = {
      .t = t,
      .transform = m_transform * Tf
    };
    m_transform = ans.pose.transform;

    // Compute twist

    Eigen::Vector2d translation = Tf.topRightCorner<2,1>();
    double rotation = std::atan2(Tf(1,0), Tf(0,0));
    double deltaT = std::chrono::duration<double>(t - m_old_data->t).count();

    ans.twist = {
      .t = get_twist_t(t, m_old_data->t, twist_timestamp_mode),
      .linear = translation / deltaT,
      .angular = rotation / deltaT
    };
  } else {
    m_old_data.emplace(std::move(dp), t);
    ans = Odometry {
      .pose = {
        .t = t,
        .transform = Eigen::Matrix3d::Identity()
      },
      .twist = std::nullopt
    };
  }

  m_old_data.emplace(std::move(dp), t);
  return ans;
}

} // namespace scan_matching_odometry