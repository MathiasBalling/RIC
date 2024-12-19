cmake_minimum_required(VERSION 3.5)
project(geographic_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

set(msg_files
  "msg/GeoPoint.msg"
  "msg/BoundingBox.msg"
  "msg/GeographicMapChanges.msg"
  "msg/GeographicMap.msg"
  "msg/GeoPose.msg"
  "msg/KeyValue.msg"
  "msg/MapFeature.msg"
  "msg/RouteNetwork.msg"
  "msg/RoutePath.msg"
  "msg/RouteSegment.msg"
  "msg/WayPoint.msg"
)
set(srv_files
  "srv/GetGeographicMap.srv"
  "srv/GetRoutePlan.srv"
  "srv/UpdateGeographicMap.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES unique_identifier_msgs std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
