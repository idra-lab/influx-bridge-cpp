#ifdef WITH_TURTLESIM

#include <influxdb_bridge_cpp/msgs/turtlesim.hpp>
#include <influxdb_bridge_cpp/point.hpp>

#include <influxdb_bridge_cpp/point.hxx>

REGISTER_MEASUREMENT(
    turtlesim::msg::Pose::SharedPtr,
    "turtlesim_pose",
    p.add_key("x", data->x).add_key("y", data->y).add_key("theta", data->theta));
#endif
