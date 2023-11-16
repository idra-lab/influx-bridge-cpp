#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <fstream>

#include <influxdb_bridge_cpp/defines.hpp>
#include <influxdb_bridge_cpp/influx_bridge_node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    // Configuration file should contains a list of lines in the form
    // <topic_name> <message_type> <measurement_name (optional)>
    std::string   config_filename = std::string(PRJ_ROOT_DIR) + "/config.txt";
    std::ifstream config_file(config_filename);
    if (!config_file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Could not open config file: %s",
                     config_filename.c_str());
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Can't configure the influxdb bridge");
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<influxdb::InfluxBridgeNode>();
    // Setup subscriptions
    for (std::string line; std::getline(config_file, line);) {
        std::istringstream iss(line);
        std::string        topic, msg_type, measurement;
        iss >> topic >> msg_type >> measurement;
        if (measurement == "") node->add_subscriber(topic, msg_type);
        else node->add_subscriber(topic, msg_type, measurement);
    }
    config_file.close();

    rclcpp::spin(node);
    node->unsubscribe_all();

    std::string export_filename = std::string(PRJ_ROOT_DIR) + "/points.txt";
    std::string fields_filename = std::string(PRJ_ROOT_DIR) + "/fields.txt";
    std::string extra_fields    = "";
    // Extra fields can be specified in the file fields.txt as a list of lines like
    // <field_name> = <field_value>

    std::ifstream fields_file(fields_filename);
    if (fields_file.is_open()) {
        for (std::string line; std::getline(fields_file, line);) {
            extra_fields += "," + line;
        }
        fields_file.close();
        // Remove spaces inside the string
        extra_fields.erase(
            std::remove_if(extra_fields.begin(), extra_fields.end(), ::isspace),
            extra_fields.end());
    }

    std::ofstream export_file(export_filename);
    RCLCPP_INFO(node->get_logger(), "exporting points to %s", export_filename.c_str());
    if (extra_fields != "") {
        RCLCPP_INFO(node->get_logger(), "exporting with extra fields: %s",
                    extra_fields.c_str());
    }
    export_file << node->export_points(extra_fields) << std::endl;
    export_file.close();
    RCLCPP_INFO(node->get_logger(), "Points exported to InfluxDB");

    rclcpp::shutdown();
    return 0;
}
