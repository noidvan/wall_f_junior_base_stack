cmake_minimum_required(VERSION 3.8)
project(wall_f_junior_base_station)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt5 REQUIRED COMPONENTS Core)
find_package(Qt5 REQUIRED COMPONENTS Quick)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)

set(HDR_FILES
  include/wall_f_junior_base_station/drive_widget.h
  include/wall_f_junior_base_station/teleop_panel.h
)

foreach(header "${HDR_FILES}")
  qt5_wrap_cpp(rviz_plugins_moc_files "${header}")
endforeach()

set(SRC_FILES
  src/drive_widget.cpp
  src/teleop_panel.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${HDR_FILES} ${rviz_plugins_moc_files})

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  "rclcpp"
  "rviz_common"
  "rviz_rendering"
  "sensor_msgs"
  "rviz_default_plugins"
  "rviz2"
  "std_msgs"
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${Qt5Widgets_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${rviz_common}
  ${rviz_rendering}
  ${rviz_default_plugins}
  ${pluginlib}
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_rendering
  sensor_msgs
  rviz_default_plugins
  rviz2
  std_msgs)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_package()
