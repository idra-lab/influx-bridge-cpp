#include <chrono>
#include <string>

#include <influxdb_bridge_cpp/point.hpp>

using influxdb::Point;

Point::Point(const std::string& meas_name) : _meas_name(meas_name) {}

std::string Point::write_query(const std::string& extra_fields) const {
    std::string data = _meas_name;

    data += extra_fields + _field_string;
    data += " " + _key_string.substr(1);

    if (_timestamp_set) data += " " + _timestamp;
    return data;
}

Point& Point::set_timestamp(const long& timestamp) {
    _timestamp     = std::to_string(timestamp);
    _timestamp_set = true;
    return *this;
}

Point& Point::set_timestamp_now() {
    auto curr_ts = std::chrono::high_resolution_clock::now();
    long unix_ts =
        std::chrono::duration_cast<std::chrono::nanoseconds>(curr_ts.time_since_epoch())
            .count();
    set_timestamp(unix_ts);
    return *this;
}
