cmake_minimum_required(VERSION 3.16)
project(voxblox_rviz_plugin)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_msgs REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(glog REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MACOSX_RPATH 0)
set(OGRE_PLUGIN_PATH "rviz_ogre_vendor")

add_definitions(-Wall)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## Use the same Qt major version as rviz_common (Qt6 on Lyrical+, Qt5 on Jazzy)
## rviz_common already pulled in its Qt, so follow that instead of guessing.
if(TARGET Qt6::Widgets)
  set(QT_VERSION_MAJOR 6)
else()
  set(QT_VERSION_MAJOR 5)
endif()
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Widgets)
message(STATUS "Using Qt${QT_VERSION_MAJOR}")
## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
set(QT_LIBRARIES Qt${QT_VERSION_MAJOR}::Widgets)

## Avoid Qt signals and slots defining "emit", "slots", etc.
add_definitions(-DQT_NO_KEYWORDS)

set(HEADER_FILES
  include/voxblox_rviz_plugin/voxblox_mesh_display.h
  include/voxblox_rviz_plugin/voxblox_mesh_visual.h
)

set(SRC_FILES
  src/voxblox_mesh_display.cc
  src/voxblox_mesh_visual.cc
)

add_library(${PROJECT_NAME}
  SHARED
  ${SRC_FILES}
  ${HEADER_FILES}
)

target_include_directories(${PROJECT_NAME}  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${voxblox_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  ${voxblox_LIBRARIES}
  glog::glog
  ${voxblox_msgs_TARGETS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "VOXBLOX_RVIZ_PLUGIN_BUILDING_LIBRARY")

##########
# EXPORT #
##########

ament_export_dependencies(
  pluginlib
  Qt${QT_VERSION_MAJOR}
  rclcpp
  rviz_ogre_vendor
  voxblox_msgs
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/icons
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/voxblox.rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
