#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <scan_matching/matcher.hpp>

#include "pm_ros.hpp"
#include "parameters.hpp"

using namespace scan_matching_odometry;

static const std::map<std::string, TwistTimestampMode> twist_timestamp_mode_map {
  { "previous", TwistTimestampMode::Previous },
  { "average", TwistTimestampMode::Average },
  { "current", TwistTimestampMode::Current }
};

class MatcherNode : public rclcpp::Node {
  std::string m_frame_id;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_sub;

  std::optional<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  std::optional<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> m_pose_pub;
  std::optional<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> m_twist_pub;
  TwistTimestampMode m_twist_ts_mode;

  Matcher m_matcher;

public:
  MatcherNode() : rclcpp::Node("scan_matching_odometry"), m_matcher("pm_config.yaml")
  {
    Parameters p(this);
    m_frame_id = p.get<std::string>("frame_id", "odom");

    {
      Parameters scan_p = p.subparams("scan");
      m_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_p.get<std::string>("topic", "/scan"),
        scan_p.parse_qos("qos"),
        std::bind(&MatcherNode::on_message, this, std::placeholders::_1)
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
  }

  void publish_pose(const Pose& pose, const std::string& child_frame_id) {
    auto position = pose.transform.topRightCorner<2,1>();

    Eigen::AngleAxis<double> aa(std::atan2(pose.transform(1,0), pose.transform(0,0)), Eigen::Vector3d(0, 0, 1));
    Eigen::Quaternion<double> q(aa);
    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg.w = q.w();
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();

    if (m_pose_pub.has_value()) {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.frame_id = m_frame_id;
      pose_msg.header.stamp = rclcpp::Time(pose.t.count());
      pose_msg.pose.orientation = quat_msg;
      pose_msg.pose.position.x = position.x();
      pose_msg.pose.position.y = position.y();
      pose_msg.pose.position.z = 0;
      (*m_pose_pub)->publish(pose_msg);
    }

    if (m_tf_broadcaster.has_value()) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.frame_id = m_frame_id;
      tf_msg.header.stamp = rclcpp::Time(pose.t.count());
      tf_msg.child_frame_id = child_frame_id;
      tf_msg.transform.translation.x = position.x();
      tf_msg.transform.translation.y = position.y();
      tf_msg.transform.translation.z = 0;
      tf_msg.transform.rotation = quat_msg;
      m_tf_broadcaster->sendTransform(tf_msg);
    }
  }

  void publish_twist(const Twist& twist, const std::string& frame_id) {
    if (m_twist_pub.has_value()) {
      geometry_msgs::msg::TwistStamped twist_msg;
      twist_msg.header.stamp = rclcpp::Time(twist.t.count());
      twist_msg.header.frame_id = frame_id;
      twist_msg.twist.linear.x = twist.linear.x();
      twist_msg.twist.linear.y = twist.linear.y();
      twist_msg.twist.linear.z = 0;
      twist_msg.twist.angular.x = 0;
      twist_msg.twist.angular.y = 0;
      twist_msg.twist.angular.z = twist.angular;
      (*m_twist_pub)->publish(twist_msg);
    }
  }

  void on_message(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
    Odometry odom = m_matcher.update(
      std::chrono::nanoseconds(rclcpp::Time(msg->header.stamp).nanoseconds()),
      rosMsgToPointMatcherCloud<double>(*msg)
    );

    publish_pose(odom.pose, msg->header.frame_id);
    if (odom.twist.has_value()) {
      publish_twist(*odom.twist, msg->header.frame_id);
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MatcherNode>());
  rclcpp::shutdown();
}