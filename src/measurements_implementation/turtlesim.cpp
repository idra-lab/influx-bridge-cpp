#ifdef WITH_TURTLESIM

#include <influxdb_bridge_cpp/msgs/turtlesim.hpp>
#include <influxdb_bridge_cpp/point.hpp>
#include <std_msgs/msg/string.hpp>

IMPLEMENT_MEASUREMENT(turtlesim::msg::Pose,
                      "turtlesim_pose",
                      point.add_key<decltype(message->x)>("x", message->x)
                          .add_key<decltype(message->y)>("y", message->y)
                          .add_key<decltype(message->theta)>("theta", message->theta));

#endif
