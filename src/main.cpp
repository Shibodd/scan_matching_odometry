#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "pm_ros.hpp"
#include "parameters.hpp"

class Matcher : public rclcpp::Node {
  std::string m_frame_id;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_sub;

  std::optional<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  std::optional<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> m_pub;

  std::optional<PointMatcher<double>::DataPoints> m_old_dp;
  Eigen::Matrix<double, 3, 3> m_pose;

public:
  Matcher() : rclcpp::Node("scan_matching_odometry")
  {
    Parameters p(this);
    m_frame_id = p.get<std::string>("frame_id", "scan_matching_odom");

    Parameters tf_p = p.subparams("tf");
    if (tf_p.get<bool>("publish", false))
      m_tf_broadcaster.emplace(*this);

    Parameters pub_p = p.subparams("pose");
    if (pub_p.get<bool>("publish", true))
      m_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        pub_p.get<std::string>("topic", "/scan_matching_odom"),
        pub_p.parse_qos("qos")
      );

    Parameters scan_p = p.subparams("scan");
    m_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_p.get<std::string>("topic", "/scan"),
      scan_p.parse_qos("qos"),
      std::bind(&Matcher::on_message, this, std::placeholders::_1)
    );
    
    m_pose.setIdentity();
  }

  void publish(const builtin_interfaces::msg::Time& stamp, const std::string& child_frame_id) {
    auto position = m_pose.topRightCorner<2,1>();

    Eigen::AngleAxis<double> aa(std::atan2(m_pose(1,0), m_pose(0,0)), Eigen::Vector3d(0, 0, 1));
    Eigen::Quaternion<double> q(aa);
    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg.w = q.w();
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();

    if (m_pub.has_value()) {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.frame_id = m_frame_id;
      pose_msg.header.stamp = stamp;
      pose_msg.pose.orientation = quat_msg;
      pose_msg.pose.position.x = position.x();
      pose_msg.pose.position.y = position.y();
      pose_msg.pose.position.z = 0;
      (*m_pub)->publish(pose_msg);
    }

    if (m_tf_broadcaster.has_value()) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.frame_id = m_frame_id;
      tf_msg.header.stamp = stamp;
      tf_msg.child_frame_id = child_frame_id;
      tf_msg.transform.translation.x = position.x();
      tf_msg.transform.translation.y = position.y();
      tf_msg.transform.translation.z = 0;
      tf_msg.transform.rotation = quat_msg;
      m_tf_broadcaster->sendTransform(tf_msg);
    }
  }

  void on_message(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
    auto dp = rosMsgToPointMatcherCloud<double>(*msg);

    if (m_old_dp.has_value()) {
      PointMatcher<double>::ICP icp;
      icp.setDefault();
      PointMatcher<double>::TransformationParameters T = icp(dp, *m_old_dp);
      assert(T.rows() == 3 && T.cols() == 3);

      Eigen::Ref<Eigen::Matrix3d> Tf(T);
      
      // Assert that the result is a rigid transformation
      assert((Tf(0,0) == Tf(1,1)) && (Tf(1,0) == -Tf(0,1)));
      assert(std::abs(Tf.topLeftCorner<2,2>().determinant() - 1) < 1e-8);
      assert((Tf.bottomLeftCorner<1,3>() == Eigen::Matrix<double, 1, 3>(0, 0, 1)));

      m_pose = m_pose * Tf;
      
      publish(msg->header.stamp, msg->header.frame_id);
    }

    m_old_dp = std::move(dp);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Matcher>());
  rclcpp::shutdown();
}
