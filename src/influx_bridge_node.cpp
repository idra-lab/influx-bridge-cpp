#include <optional>
#include <string>

#include <influxdb_bridge_cpp/influx_bridge_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

using influxdb::InfluxBridgeNode;

InfluxBridgeNode::InfluxBridgeNode(std::optional<std::string> node_name) :
        Node(node_name.value_or("influxdb_bridge_node")) {}

std::string InfluxBridgeNode::export_points(const std::string& common_fields) const {
    std::string result = "";
    for (const auto& point : _collected_points) {
        result += point.write_query(common_fields) + "\n";
    }
    return result;
}

#define REGISTER_NEW_MSG_TYPE(type_literal, type_namespace)                              \
    if (topic_type == type_literal) {                                                    \
        if (subcriber_added) {                                                           \
            RCLCPP_ERROR(this->get_logger(), "Trying to subscribe to topic %s twice",    \
                         topic_name.c_str());                                            \
        }                                                                                \
        RCLCPP_INFO(this->get_logger(),                                                  \
                    "Subscribing to topic %s with measurement named \"%s\"",             \
                    topic_name.c_str(), measure_name.value_or("<default>").c_str());     \
        auto lambda = [this, measure_name](const type_namespace::SharedPtr msg) {        \
            this->topic_callback<type_namespace::SharedPtr>(msg, measure_name);          \
        };                                                                               \
        auto sub = this->create_subscription<type_namespace>(topic_name, 1, lambda);     \
        _subscribers.push_back(sub);                                                     \
        subcriber_added = true;                                                          \
    }

void InfluxBridgeNode::unsubscribe_all() {
    for (auto& sub : _subscribers) { sub.reset(); }
    _subscribers.clear();
}

void InfluxBridgeNode::add_subscriber(std::string                topic_name,
                                      std::string                topic_type,
                                      std::optional<std::string> measure_name) {
    bool subcriber_added = false;

#ifdef WITH_TURTLESIM
    REGISTER_NEW_MSG_TYPE("turtlesim/Pose", turtlesim::msg::Pose)
#endif

    if (!subcriber_added) {
        RCLCPP_ERROR(this->get_logger(), "Failed to add subscriber to topic %s",
                     topic_name.c_str());
    }
}
