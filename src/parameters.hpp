#ifndef CONTROLNODE_PARAMETERS_HPP
#define CONTROLNODE_PARAMETERS_HPP

#include <rclcpp/parameter_value.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include <rmw/qos_string_conversions.h>
#include <rmw/time.h>
#include <rmw/types.h>
#include <string>
#include <rclcpp/rclcpp.hpp>


template<typename T>
struct is_std_vector : std::false_type {};

template<typename T>
struct is_std_vector<std::vector<T>> : std::true_type {};

class Parameters {
  rclcpp::Node& m_node;
  std::string m_prefix;

  inline std::string get_param_name(const std::string& s) const {
    return m_prefix.size() > 0?
      m_prefix + "." + s
      : s;
  }

  template <typename T>
  inline T log_value(const std::string& name, T val) const {
    std::string repr;

    if constexpr (std::is_same_v<T, std::string>)
      repr = val;
    else if constexpr (std::is_same_v<T, bool>)
      repr = val? "true" : "false";
    else if constexpr (is_std_vector<T>::value) {
      for (auto& v : val)
        repr += std::to_string(v) + " ";
    }
    else
      repr = std::to_string(val);

    RCLCPP_INFO(m_node.get_logger(), "Parameter %s: %s", name.c_str(), repr.c_str());
    return val;
  }

  inline std::nullopt_t log_missing_value(const std::string& name) const {
    RCLCPP_INFO(m_node.get_logger(), "Parameter %s: NOT SET", name.c_str());
    return std::nullopt;
  }

public:
  Parameters(rclcpp::Node* node, const std::string& prefix = "") : m_node(*node), m_prefix(prefix) {}

  template <typename T>
  T get(const std::string& s, std::optional<T> default_value = std::nullopt) const {
    auto name = get_param_name(s);

    try {
      if (m_node.has_parameter(name))
        return log_value<T>(name, m_node.get_parameter(name).get_value<T>());

      if (default_value.has_value()) {
        auto param = m_node.declare_parameter(name, rclcpp::ParameterValue(T{}).get_type());
        
        T ans;
        if (m_node.get_parameter(name, ans))
          return log_value<T>(name, ans);
        else
          return log_value<T>(name, *default_value);
      }
      else
        return log_value<T>(name, m_node.declare_parameter(name, rclcpp::ParameterValue(T{}).get_type()).get<T>());
    } catch (...) {
      RCLCPP_ERROR(m_node.get_logger(), "Error while parsing parameter '%s'.", name.c_str());
      throw;
    }
  }

  template <typename T>
  std::optional<T> get_maybe(const std::string& s) const {
    auto name = get_param_name(s);

    try {
      if (m_node.has_parameter(name))
        return log_value<T>(name, m_node.get_parameter(name).get_value<T>());

      m_node.declare_parameter(name, rclcpp::ParameterValue(T{}).get_type());

      T ans;
      if (m_node.get_parameter(name, ans))
        return log_value<T>(name, ans);
      else
        return log_missing_value(name);
    } catch (...) {
      RCLCPP_ERROR(m_node.get_logger(), "Error while parsing parameter '%s'.", name.c_str());
      throw;
    }
  }

  std::array<float, 3> parse_rgba(const std::string& prefix, float& alpha) const {
    const Parameters p = subparams(prefix);
    alpha = (float)p.get<float>("a");
    return { (float)p.get<double>("r"), (float)p.get<double>("g"), (float)p.get<double>("b") };
  }

  rmw_time_t parse_rmw_time(const std::string& prefix, std::optional<rmw_time_t> default_value) const {
    const Parameters p = subparams(prefix);
    rmw_time_t s;
    s.sec = p.get<int>("sec", default_value.has_value()? std::optional<int>(default_value->sec) : std::nullopt);
    s.nsec = p.get<int>("nsec", default_value.has_value()? std::optional<int>(default_value->nsec) : std::nullopt);
    return s;
  }

  rclcpp::QoS parse_qos(const std::string& prefix, rmw_qos_profile_t default_qos = rmw_qos_profile_default) const {
    const Parameters p = subparams(prefix);

    rmw_qos_profile_t profile;
    profile.history = rmw_qos_history_policy_from_str(p.get<std::string>("history", rmw_qos_history_policy_to_str(default_qos.history)).c_str());
    profile.reliability = rmw_qos_reliability_policy_from_str(p.get<std::string>("reliability", rmw_qos_reliability_policy_to_str(default_qos.reliability)).c_str());
    profile.durability = rmw_qos_durability_policy_from_str(p.get<std::string>("durability", rmw_qos_durability_policy_to_str(default_qos.durability)).c_str());
    profile.liveliness = rmw_qos_liveliness_policy_from_str(p.get<std::string>("liveliness", rmw_qos_liveliness_policy_to_str(default_qos.liveliness)).c_str());
    profile.depth = p.get<int>("depth", default_qos.depth);
    profile.deadline = p.parse_rmw_time("deadline", default_qos.deadline);
    profile.lifespan = p.parse_rmw_time("lifespan", default_qos.lifespan);
    profile.liveliness_lease_duration = p.parse_rmw_time("liveliness_lease_duration", default_qos.liveliness_lease_duration);
    profile.avoid_ros_namespace_conventions = p.get<bool>("avoid_ros_namespace_conventions", default_qos.avoid_ros_namespace_conventions);

    return rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(profile), profile);
  }

  template<typename SentinelFieldT>
  void parse_list(const std::string& prefix, const std::string& sentinel_field, std::function<void(int, const Parameters&)> parser) const {
    const Parameters p = subparams(prefix);
    for (int i = 0; ; ++i) {
      const Parameters p_i = p.subparams(std::to_string(i));

      if (p_i.get_maybe<SentinelFieldT>(sentinel_field))
        parser(i, p_i);
      else
        return;
    }
  }

  Parameters subparams(const std::string& prefix) const {
    return Parameters(&m_node, get_param_name(prefix));
  }
};

#endif // !CONTROLNODE_PARAMETERS_HPP