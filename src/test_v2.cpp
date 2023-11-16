#include <iostream>
#include <vector>

#include <influxdb_bridge_cpp/point.hpp>

int main() {
    float            v1 = 1.0;
    double           v2 = 1.0;
    long double      v3 = 1.0;
    int              v4 = 2;
    std::vector<int> v5 = {1, 2, 3, 4, 5};
    influxdb::Point  p("test");

    // p.add_key(MAKE_VALUEPAIR("key1", v1));
    // p.add_key(MAKE_VALUEPAIR("key2", v2));
    // p.add_key(MAKE_VALUEPAIR("key3", v3));
    // p.add_key(MAKE_VALUEPAIR("key4", v4));
    p.add_key<decltype(v4)>("key4", v4);
    p.add_key(MAKE_VALUEPAIR("vector", v5));

    std::cout << p.write_query() << std::endl;
    return 0;
}
