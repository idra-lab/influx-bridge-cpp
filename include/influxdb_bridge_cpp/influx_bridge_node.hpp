#ifndef INFLUXDBBRIDGE_ROSNODE_HPP__
#define INFLUXDBBRIDGE_ROSNODE_HPP__

#include <any>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <influxdb_bridge_cpp/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace influxdb {

    class InfluxBridgeNode : public rclcpp::Node {
    private:
        // https://stackoverflow.com/questions/26843024/array-of-shared-pointers-to-different-classes
        std::vector<std::any> _subscribers;
        std::vector<Point>    _collected_points;

        template <typename T>
        void topic_callback(
            const T msg, const std::optional<std::string> measure_name = std::nullopt) {
            _collected_points.push_back(create_measurement_point<T>(
                msg, measure_name, this->get_clock()->now().nanoseconds()));
        }

    public:
        InfluxBridgeNode(std::optional<std::string> node_name = std::nullopt);

        void        add_subscriber(std::string                topic_name,
                                   std::string                topic_type,
                                   std::optional<std::string> measure_name = std::nullopt);
        void        unsubscribe_all();
        std::string export_points(const std::string& common_fields = "") const;
    };

}  // namespace influxdb

#endif  // INFLUXDBBRIDGE_ROSNODE_HPP__
