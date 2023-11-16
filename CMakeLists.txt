cmake_minimum_required(VERSION 3.8)
project(influxdb-bridge-cpp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# The following macro is used to add dependencies to the influxdb-bridge executable only if
# packages are found.
# In that case rules for storing messages associated to the executable are compiled and
# thus available to the executable.
# Compilation of some parts are ruled by the flag WITH_<package_name> which is defined
# that is define here only if the package is found.
macro(bridge_msg_compiler package_name)
    find_package(${package_name})
    if (${package_name}_FOUND)
        ament_target_dependencies(influxdb-bridge ${package_name})
        string(TOUPPER ${package_name} package_name_upper)
        target_compile_definitions(influxdb-bridge PRIVATE "WITH_${package_name_upper}")
    endif()
endmacro()

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/influxdb_bridge_cpp/defines.hpp.in
    ${CMAKE_CURRENT_SOURCE_DIR}/include/influxdb_bridge_cpp/defines.hpp
    )

add_executable(influxdb-bridge src/main.cpp)
target_include_directories(influxdb-bridge PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
file(GLOB_RECURSE MEAS_IMPLEMENTATIONS ${CMAKE_CURRENT_SOURCE_DIR}/src/measurements_implementation/*.cpp)
target_sources(influxdb-bridge
    PRIVATE
    src/influx_bridge_node.cpp
    src/point.cpp
    ${MEAS_IMPLEMENTATIONS}
    )

ament_target_dependencies(influxdb-bridge
    rclcpp
    )
bridge_msg_compiler(std_msgs)
bridge_msg_compiler(turtlesim)

add_executable(testnode
    src/test_v2.cpp
    src/point.cpp
    )
target_include_directories(testnode PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

install(TARGETS
    influxdb-bridge
    testnode
    DESTINATION lib/${PROJECT_NAME}
    )
install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}
    )

ament_package()
