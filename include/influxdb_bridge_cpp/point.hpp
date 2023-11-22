#ifndef INFLUXDBBRIDGE_POINT_HPP__
#define INFLUXDBBRIDGE_POINT_HPP__

#include <cstddef>
#include <optional>
#include <string>
#include <utility>

namespace influxdb {

    template <typename T>
    using Valuepair = std::pair<std::string, const T&>;

    class Point {
    private:
        bool        _timestamp_set;
        std::string _timestamp;
        std::string _meas_name;
        std::string _key_string;
        std::string _field_string;

    public:
        Point(const std::string& meas_name);

        std::string write_query(const std::string& extra_fields = "") const;

        Point& set_timestamp(const long& timestamp);
        Point& set_timestamp_now();

        template <typename T>
        Point& add_key(const Valuepair<T>& pair);

        template <typename T>
        Point& add_field(const Valuepair<T>& pair);

        template <typename T>
        Point& add_key(const std::string& label, const T& value);

        template <typename T>
        Point& add_field(const std::string& label, const T& value);
    };

    // The following function creates the common interface that must be implemented for
    // each message type that should be exported to InfluxDB.
    // Take a look at IMPLEMENT_MEASUREMENT at the bottom of the file for a macro that
    // makes the implementation easier.
    template <typename T>
    Point create_measurement_point(const T&                   message,
                                   std::optional<std::string> measure_name = std::nullopt,
                                   const std::optional<long>& timestamp = std::nullopt);

};  // namespace influxdb

// Convenience macro for creating Valuepairs
#define MAKE_VALUEPAIR(name, value)                                                      \
    influxdb::Valuepair<decltype(value)> { name, value }

// Convenience macro for mapping ROS messages to InfluxDB points
// "type" is the ROS message type as from the header file (e.g. turtlesim::msg::Pose)
// "default_name" is the default name of the point when inserted on the database
// "implementation" is the body that needs to be implemented for each type. A Point
// object "point" is automatically created and returned, so only proper calls to
// "point.add_key" and "point.add_field" are needed.
#define IMPLEMENT_MEASUREMENT(type, default_name, implementation)                        \
    template <>                                                                          \
    influxdb::Point influxdb::create_measurement_point<type::SharedPtr>(                 \
        const type::SharedPtr& message, std::optional<std::string> measure_name,         \
        const std::optional<long>& timestamp) {                                          \
        influxdb::Point point = influxdb::Point(measure_name.value_or(default_name));    \
        implementation;                                                                  \
        if (timestamp.has_value()) { point.set_timestamp(timestamp.value()); }           \
        return point;                                                                    \
    }

#include <influxdb_bridge_cpp/point.hxx>

#endif  // INFLUXDBBRIDGE_POINT_HPP__
