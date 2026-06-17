cmake_minimum_required(VERSION 3.8)
project(falcon_mapping_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(mapping_node
        src/mapping_node.cpp
        src/tsdf_integrator.cpp
        src/esdf_integrator.cpp
)

ament_target_dependencies(mapping_node
        rclcpp
        sensor_msgs
        nav_msgs
        visualization_msgs
)

install(TARGETS mapping_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
