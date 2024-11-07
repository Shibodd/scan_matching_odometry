#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "pm_ros.hpp"
#include "parameters.hpp"

enum class TwistTimestampMode {
  Previous,
  Average,
  Current
};

static const std::map<std::string, TwistTimestampMode> twist_timestamp_mode_map {
  { "previous", TwistTimestampMode::Previous },
  { "average", TwistTimestampMode::Average },
  { "current", TwistTimestampMode::Current }
};

class Matcher : public rclcpp::Node {
  std::string m_frame_id;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_sub;

  std::optional<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  std::optional<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> m_pose_pub;
  std::optional<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> m_twist_pub;
  TwistTimestampMode m_twist_ts_mode;

  struct OldData {
    PointMatcher<double>::DataPoints dp;
    std::chrono::nanoseconds t;
  };

  std::optional<OldData> m_old_data;
  Eigen::Matrix<double, 3, 3> m_pose;

public:
  Matcher() : rclcpp::Node("scan_matching_odometry")
  {
    Parameters p(this);
    m_frame_id = p.get<std::string>("frame_id", "scan_matching_odom");

    {
      Parameters scan_p = p.subparams("scan");
      m_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_p.get<std::string>("topic", "/scan"),
        scan_p.parse_qos("qos"),
        std::bind(&Matcher::on_message, this, std::placeholders::_1)
      );
    }

    {
      Parameters tf_p = p.subparams("tf");
      if (tf_p.get<bool>("publish", false))
        m_tf_broadcaster.emplace(*this);
    }

    {
      Parameters pose_p = p.subparams("pose");
      if (pose_p.get<bool>("publish", true))
        m_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
          pose_p.get<std::string>("topic", "/scan_matching/pose"),
          pose_p.parse_qos("qos")
        );
    }

    {
      Parameters twist_p = p.subparams("twist");
      if (twist_p.get<bool>("publish", true)) {
        m_twist_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
          twist_p.get<std::string>("topic", "/scan_matching/twist"),
          twist_p.parse_qos("qos")
        );

        auto ts_mode_string = twist_p.get<std::string>("timestamp_from_scan", "previous");
        auto ts_mode_it = twist_timestamp_mode_map.find(ts_mode_string);
        if (ts_mode_it == twist_timestamp_mode_map.end()) {
          RCLCPP_FATAL(this->get_logger(), "Invalid timestamp_mode %s. Allowed values: previous, average, current", ts_mode_string.c_str());
          throw std::invalid_argument("timestamp_mode");
        }
        m_twist_ts_mode = ts_mode_it->second;
      }
    }
    
    m_pose.setIdentity();
  }

  void publish_pose(std::chrono::nanoseconds t, const std::string& child_frame_id) {
    auto position = m_pose.topRightCorner<2,1>();

    Eigen::AngleAxis<double> aa(std::atan2(m_pose(1,0), m_pose(0,0)), Eigen::Vector3d(0, 0, 1));
    Eigen::Quaternion<double> q(aa);
    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg.w = q.w();
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();

    if (m_pose_pub.has_value()) {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.frame_id = m_frame_id;
      pose_msg.header.stamp = rclcpp::Time(t.count());
      pose_msg.pose.orientation = quat_msg;
      pose_msg.pose.position.x = position.x();
      pose_msg.pose.position.y = position.y();
      pose_msg.pose.position.z = 0;
      (*m_pose_pub)->publish(pose_msg);
    }

    if (m_tf_broadcaster.has_value()) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.frame_id = m_frame_id;
      tf_msg.header.stamp = rclcpp::Time(t.count());
      tf_msg.child_frame_id = child_frame_id;
      tf_msg.transform.translation.x = position.x();
      tf_msg.transform.translation.y = position.y();
      tf_msg.transform.translation.z = 0;
      tf_msg.transform.rotation = quat_msg;
      m_tf_broadcaster->sendTransform(tf_msg);
    }
  }

  void publish_twist(std::chrono::nanoseconds t, const std::string& frame_id, const Eigen::Vector2d& linear, double angular) {
    if (m_twist_pub.has_value()) {
      geometry_msgs::msg::TwistStamped twist_msg;
      twist_msg.header.stamp = rclcpp::Time(t.count());
      twist_msg.header.frame_id = frame_id;
      twist_msg.twist.linear.x = linear.x();
      twist_msg.twist.linear.y = linear.y();
      twist_msg.twist.linear.z = 0;
      twist_msg.twist.angular.x = 0;
      twist_msg.twist.angular.y = 0;
      twist_msg.twist.angular.z = angular;
      (*m_twist_pub)->publish(twist_msg);
    }
  }

  void on_message(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
    auto dp = rosMsgToPointMatcherCloud<double>(*msg);
    std::chrono::nanoseconds t(rclcpp::Time(msg->header.stamp).nanoseconds());

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
      
      m_pose = m_pose * Tf;
      publish_pose(t, msg->header.frame_id);

      Eigen::Vector2d translation = Tf.topRightCorner<2,1>();
      double rotation = std::atan2(Tf(1,0), Tf(0,0));
      double deltaT = std::chrono::duration<double>(t - m_old_data->t).count();
      std::chrono::nanoseconds twist_timestamp;

      switch (m_twist_ts_mode) {
        case TwistTimestampMode::Previous:
          twist_timestamp = m_old_data->t;
          break;
        case TwistTimestampMode::Average:
          twist_timestamp = m_old_data->t + (t - m_old_data->t) / 2;
          break;
        case TwistTimestampMode::Current:
          twist_timestamp = t;
          break;
        default:
          assert(false);
      }

      publish_twist(m_old_data->t, msg->header.frame_id, translation / deltaT, rotation / deltaT);
    }

    m_old_data.emplace(std::move(dp), t);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Matcher>());
  rclcpp::shutdown();
}