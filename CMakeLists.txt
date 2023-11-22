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

# Configuration of the "defines.hpp " file
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/influxdb_bridge_cpp/defines.hpp.in
    ${CMAKE_CURRENT_SOURCE_DIR}/include/influxdb_bridge_cpp/defines.hpp
    )


#  ___        __ _            ____  ____    ____       _     _
# |_ _|_ __  / _| |_   ___  _|  _ \| __ )  | __ ) _ __(_) __| | __ _  ___
#  | || '_ \| |_| | | | \ \/ / | | |  _ \  |  _ \| '__| |/ _` |/ _` |/ _ \
#  | || | | |  _| | |_| |>  <| |_| | |_) | | |_) | |  | | (_| | (_| |  __/
# |___|_| |_|_| |_|\__,_/_/\_\____/|____/  |____/|_|  |_|\__,_|\__, |\___|
#                                                              |___/


add_executable(influxdb-bridge src/main.cpp)
target_include_directories(
    influxdb-bridge
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

# Other implementation-related source files
file(GLOB_RECURSE MEAS_IMPLEMENTATIONS ${CMAKE_CURRENT_SOURCE_DIR}/src/measurements_implementation/*.cpp)
target_sources(
    influxdb-bridge
    PRIVATE
    src/influx_bridge_node.cpp
    src/point.cpp
    ${MEAS_IMPLEMENTATIONS}
    )

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
        message(STATUS "Compiling message from package ${package_name}")
    endif()
endmacro()

ament_target_dependencies(influxdb-bridge rclcpp)
set(COMPILABLE_MESSAGE_PACKAGES
    std_msgs
    sensor_msgs
    turtlesim
    )
foreach(package_name ${COMPILABLE_MESSAGE_PACKAGES})
    bridge_msg_compiler(${package_name})
endforeach()

#  ___           _        _ _
# |_ _|_ __  ___| |_ __ _| | |
#  | || '_ \/ __| __/ _` | | |
#  | || | | \__ \ || (_| | | |
# |___|_| |_|___/\__\__,_|_|_|
#
install(
    TARGETS influxdb-bridge
    DESTINATION lib/${PROJECT_NAME}
    )
install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}
    )

ament_package()
