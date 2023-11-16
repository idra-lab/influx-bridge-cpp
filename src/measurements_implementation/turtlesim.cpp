#ifdef WITH_TURTLESIM

#include <influxdb_bridge_cpp/msgs/turtlesim.hpp>
#include <influxdb_bridge_cpp/point.hpp>
#include <std_msgs/msg/string.hpp>

IMPLEMENT_MEASUREMENT(turtlesim::msg::Pose,
                      "turtlesim_pose",
                      point.add_key<decltype(message->x)>("x", message->x)
                          .add_key<decltype(message->y)>("y", message->y)
                          .add_key<decltype(message->theta)>("theta", message->theta)
                          .set_timestamp_now());

// template <>
// influxdb::Point influxdb::create_measurement_point<std_msgs::msg::String>(const
// std_msgs::msg::String& message,
//         std::optional<std::string> measure_name) {
//     influxdb::Point p = influxdb::Point(measure_name.value_or("std_msgs_string"));
//     p.add_field("data", std::string(message.data));
//     return p;
// }

// template <>
// influxdb::Point influxdb::create_measurement_point<turtlesim::msg::Pose::SharedPtr>(
//     const turtlesim::msg::Pose::SharedPtr& message, std::optional<std::string>
//     measure_name) { influxdb::Point p =
//     influxdb::Point(measure_name.value_or("turtlesim_pose"));
//     p.add_key<decltype(message->x)>("x", message->x);
//     return p;
// }
#endif
