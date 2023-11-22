#include <std_msgs/msg/detail/string__struct.hpp>
#ifdef WITH_STD_MSGS

#include <influxdb_bridge_cpp/msgs/std_msgs.hpp>
#include <influxdb_bridge_cpp/point.hpp>

#include <influxdb_bridge_cpp/point.hxx>

IMPLEMENT_MEASUREMENT(std_msgs::msg::String,
                      "string",
                      point.add_key<decltype(message->data)>("content", message->data););
IMPLEMENT_MEASUREMENT(std_msgs::msg::Float64MultiArray,
                      "multi_array",
                      point.add_key<decltype(message->data)>("content", message->data););

#endif
