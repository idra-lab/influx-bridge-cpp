#ifndef INFLUXDBBDRIDGE_MSG_SENSOR_MSGS_HPP__
#define INFLUXDBBDRIDGE_MSG_SENSOR_MSGS_HPP__

#include <influxdb_bridge_cpp/point.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>

#ifdef WITH_SENSOR_MSGS
#include <sensor_msgs/msg/joint_state.hpp>

// template <>
// influxdb::Point
// influxdb::create_measurement_point<sensor_msgs::msg::JointState::SharedPtr>(
//     const sensor_msgs::msg::JointState::SharedPtr&,
//     std::optional<std::string> measure_name,
//     const std::optional<long>& timestamp);

#endif

#endif  // INFLUXDBBDRIDGE_MSG_SENSOR_MSGS_HPP__
