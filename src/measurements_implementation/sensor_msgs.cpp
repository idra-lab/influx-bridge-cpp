#ifdef WITH_STD_MSGS

#include <influxdb_bridge_cpp/msgs/sensor_msgs.hpp>
#include <influxdb_bridge_cpp/point.hpp>

#include <influxdb_bridge_cpp/point.hxx>

IMPLEMENT_MEASUREMENT(sensor_msgs::msg::JointState,
                      "JointState",
                      point.add_key<int>("seq", message->header.stamp.sec);
                      point.add_key<decltype(message->position)>("q", message->position);
                      point.add_key<decltype(message->velocity)>("qdot",
                                                                 message->velocity);
                      point.add_key<decltype(message->effort)>("tau", message->effort););

#endif
