#include <filesystem>

#include <ranges>
#include <rclcpp/rclcpp.hpp>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>

#include <scan_matching/matcher.hpp>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>


#include "pm_ros.hpp"

using namespace scan_matching_odometry;

struct PointFactoryBase {
  struct Points {
    std::chrono::nanoseconds t;
    PointMatcher<double>::DataPoints dp;
  };

  virtual Points operator()(const rosbag2_storage::SerializedBagMessageSharedPtr&) = 0;
};

struct LaserScanPointFactory : public PointFactoryBase {
  rclcpp::Serialization<sensor_msgs::msg::LaserScan> m_serialization;

  virtual Points operator()(const rosbag2_storage::SerializedBagMessageSharedPtr& msg) override {
    sensor_msgs::msg::LaserScan scan;

    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    m_serialization.deserialize_message(&serialized_msg, &scan);

    return Points {
      .t = std::chrono::nanoseconds(rclcpp::Time(scan.header.stamp).nanoseconds()),
      .dp = rosMsgToPointMatcherCloud<double>(scan)
    };
  }
};

class OdometryFromBag {
  std::optional<Odometry> m_odometry;
  Matcher m_matcher;
  std::unique_ptr<rosbag2_cpp::Reader> m_reader;
  PointFactoryBase& m_point_factory;

  TwistTimestampMode m_twist_t_mode;

public:
  using value_type = Odometry;
  using difference_type = std::ptrdiff_t;
  class sentinel {};

  const value_type& operator*() const { return *m_odometry; }
  const value_type& operator->() const { return *m_odometry; }

  OdometryFromBag(const std::filesystem::path& path_to_bag, const std::string_view& topic, PointFactoryBase& point_factory, TwistTimestampMode twist_t_mode = TwistTimestampMode::Average)
      : m_point_factory(point_factory), 
        m_twist_t_mode(twist_t_mode)
  {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = path_to_bag;

    m_reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
    m_reader->open(storage_options);

    rosbag2_storage::StorageFilter filter;
    filter.topics.emplace_back(topic);
    m_reader->set_filter(filter);

    ++(*this);
  }

  OdometryFromBag& operator++() {
    // Reader is closed or bag is exhausted
    if (m_reader == nullptr or !m_reader->has_next()) {
      m_odometry.reset();
      m_reader.reset();
      return *this;
    }

    auto pts = m_point_factory(m_reader->read_next());
    m_odometry.emplace(m_matcher.update(pts.t, pts.dp, m_twist_t_mode));
    return *this;
  }

  void operator++(int) { ++*this; }

  OdometryFromBag& operator=(OdometryFromBag&& other) {
    m_odometry.swap(other.m_odometry);
    m_matcher = std::move(other.m_matcher);
    m_reader.swap(other.m_reader);
    m_point_factory = other.m_point_factory;
    m_twist_t_mode = other.m_twist_t_mode;
    return *this;
  }

  OdometryFromBag(OdometryFromBag&& other) :
    m_odometry(std::move(other.m_odometry)),
    m_matcher(std::move(other.m_matcher)),
    m_reader(std::move(other.m_reader)),
    m_point_factory(other.m_point_factory),
    m_twist_t_mode(other.m_twist_t_mode)
  {
  }

  bool operator==(const sentinel&) const { return not m_odometry.has_value(); }
  operator bool() const { return m_odometry.has_value(); }
};

#include <ranges>

std::ranges::subrange<OdometryFromBag, OdometryFromBag::sentinel> read_bag(const std::filesystem::path& path_to_bag, const std::string_view& topic, PointFactoryBase& point_factory, TwistTimestampMode ttm = TwistTimestampMode::Average) {
  return {
    OdometryFromBag(path_to_bag, topic, point_factory, ttm),
    OdometryFromBag::sentinel{}
  };
}

#include <fstream>

static const std::map<std::string, TwistTimestampMode> twist_timestamp_mode_map {
  { "previous", TwistTimestampMode::Previous },
  { "average", TwistTimestampMode::Average },
  { "current", TwistTimestampMode::Current }
};

int main(int argc, const char* argv[]) {
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0] << " topic bag_path out_path twist_timestamp_mode" << std::endl;
    return 1;
  }

  const char* topic = argv[1];
  const std::filesystem::path bag_path = argv[2];
  const std::filesystem::path out_path = argv[3];
  const char* twist_timestamp_mode = argv[4];

  auto ttm_it = twist_timestamp_mode_map.find(twist_timestamp_mode);
  if (ttm_it == twist_timestamp_mode_map.end()) {
    std::cerr << "Allowed values for twist_timestamp_mode: " << std::endl;
    for (auto& kvp : twist_timestamp_mode_map)
      std::cerr << kvp.first << std::endl;
    return 1;
  }
  TwistTimestampMode ttm = ttm_it->second;

  LaserScanPointFactory pf;
  auto bag = read_bag(bag_path, topic, pf, ttm);

  std::ofstream f(out_path);
  f << "x,y,phi,vx,vy,phidot" << std::endl;

  for (const Odometry& odom : bag) {
    bool twist = odom.twist.has_value();

    auto twist_t = twist? odom.twist->t.count() : 0;
    auto pose_t = twist? odom.pose.t.count() : 0;
    double x = odom.pose.transform(0,2);
    double y = odom.pose.transform(1,2);
    double phi = std::atan2(odom.pose.transform(1,0), odom.pose.transform(0,0));
    double vx = twist? odom.twist->linear.x() : 0;
    double vy = twist? odom.twist->linear.y() : 0;
    double phidot = twist? odom.twist->angular : 0;

    f << pose_t << "," << x << "," << y << "," << phi << "," << twist_t << "," << vx << "," << vy << "," << phidot << "\n";
  }

  return 0;
}