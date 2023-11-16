#ifndef INFLUXDBBRIDGE_POINT_HXX__
#define INFLUXDBBRIDGE_POINT_HXX__

#include <any>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include <influxdb_bridge_cpp/point.hpp>

using influxdb::Point;
using influxdb::Valuepair;

//  _  __                                _   _____ _      _     _
// | |/ /___ _   _ ___    __ _ _ __   __| | |  ___(_) ___| | __| |___
// | ' // _ \ | | / __|  / _` | '_ \ / _` | | |_  | |/ _ \ |/ _` / __|
// | . \  __/ |_| \__ \ | (_| | | | | (_| | |  _| | |  __/ | (_| \__ \
// |_|\_\___|\__, |___/  \__,_|_| |_|\__,_| |_|   |_|\___|_|\__,_|___/
//           |___/
//
// Declaration of the template function that converts different types into strings
// for inserting Point data into InfluxDB.
template <typename T>
inline std::string valuepair_to_string(const Valuepair<T>& pair);

// Specialization for std::string
template <>
inline std::string valuepair_to_string<std::string>(const Valuepair<std::string>& pair) {
    return pair.first + "=" + pair.second;
}

// Specialization for floating point types
template <std::floating_point T>
inline std::string valuepair_to_string(const Valuepair<T>& pair) {
    return pair.first + "=" + std::to_string(pair.second);
}

// Specialization for unsigned integral (unsigned) types
template <std::unsigned_integral T>
inline std::string valuepair_to_string(const Valuepair<T>& pair) {
    return pair.first + "=" + std::to_string(pair.second) + "u";
}

// Specialization for integral (int) types
template <std::integral T>
inline std::string valuepair_to_string(const Valuepair<T>& pair) {
    return pair.first + "=" + std::to_string(pair.second) + "i";
}

// Specialization for standard vectors
template <typename T>
inline std::string valuepair_to_string(const Valuepair<std::vector<T>>& pair) {
    std::string result = "";
    for (std::size_t i = 0; i < pair.second.size(); ++i) {
        result += "," + valuepair_to_string<T>(MAKE_VALUEPAIR(
                            pair.first + std::to_string(i), pair.second[i]));
    }
    return result.substr(1);
}

// This is the last condition: if valuepair_to_string is not specialized for
// the type, then this static_assert will fail, aborting compilation.
template <typename T>
inline std::string valuepair_to_string(const Valuepair<T>&) {
    static_assert(std::is_same_v<T, void>,
                  "valuepair_to_string is not specialized for this type");
    return "";
}

template <typename T>
struct is_std_vector : std::false_type {};

template <typename T>
struct is_std_vector<std::vector<T>> : std::true_type {};

template <typename T>
Point& Point::add_key(const Valuepair<T>& pair) {
    if constexpr (is_std_vector<T>::value) {
        add_field<std::size_t>(MAKE_VALUEPAIR("vector_size", pair.second.size()));
    }
    _key_string += "," + valuepair_to_string(pair);
    return *this;
}

template <typename T>
Point& Point::add_field(const Valuepair<T>& pair) {
    _field_string += "," + valuepair_to_string(pair);
    return *this;
}

template <typename T>
Point& Point::add_key(const std::string& label, const T& value) {
    return add_key(MAKE_VALUEPAIR(label, value));
}

template <typename T>
Point& Point::add_field(const std::string& label, const T& value) {
    return add_field(MAKE_VALUEPAIR(label, value));
}

#endif  // INFLUXDBBRIDGE_POINT_HXX__
