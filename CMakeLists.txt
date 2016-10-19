cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detector)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS roscpp rviz std_msgs std_srvs geometry_msgs sensor_msgs visualization_msgs tf message_generation)
find_package(Armadillo REQUIRED)
find_package(Boost 1.54.0 REQUIRED system)

add_message_files(FILES CircleObstacle.msg SegmentObstacle.msg Obstacles.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES obstacle_detector
   CATKIN_DEPENDS roscpp rviz std_msgs std_srvs geometry_msgs sensor_msgs visualization_msgs tf message_runtime
   DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_executable(scans_merger src/scans_merger.cpp include/scans_merger.h include/utilities/math_utilities.h)
target_link_libraries(scans_merger ${catkin_LIBRARIES})
add_dependencies(scans_merger ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(obstacle_detector src/obstacle_detector.cpp include/obstacle_detector.h include/utilities/math_utilities.h
               include/figures/point.h include/figures/segment.h include/figures/circle.h include/utilities/figure_fitting.h
               include/utilities/point_set.h)
target_link_libraries(obstacle_detector ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})
add_dependencies(obstacle_detector ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(obstacle_tracker src/obstacle_tracker.cpp include/obstacle_tracker.h include/utilities/kalman.h include/utilities/tracked_obstacle.h)
target_link_libraries(obstacle_tracker ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})
add_dependencies(obstacle_tracker ${PROJECT_NAME}_gencpp)


add_executable(obstacle_visualizer src/obstacle_visualizer.cpp include/obstacle_visualizer.h)
target_link_libraries(obstacle_visualizer ${catkin_LIBRARIES})
add_dependencies(obstacle_visualizer ${PROJECT_NAME}_gencpp)


add_executable(obstacle_recorder src/obstacle_recorder.cpp include/obstacle_recorder.h)
target_link_libraries(obstacle_recorder ${catkin_LIBRARIES})
add_dependencies(obstacle_recorder ${PROJECT_NAME}_gencpp)


add_executable(static_scan_publisher src/static_scan_publisher.cpp)
target_link_libraries(static_scan_publisher ${catkin_LIBRARIES})
add_dependencies(static_scan_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(obstacle_publisher src/obstacle_publisher.cpp include/obstacle_publisher.h)
target_link_libraries(obstacle_publisher ${catkin_LIBRARIES})
add_dependencies(obstacle_publisher ${PROJECT_NAME}_gencpp)


add_library(${PROJECT_NAME}_gui src/panels/scans_merger_panel.cpp include/panels/scans_merger_panel.h
                                src/panels/obstacle_detector_panel.cpp include/panels/obstacle_detector_panel.h
                                src/panels/obstacle_tracker_panel.cpp include/panels/obstacle_tracker_panel.h
                                src/panels/obstacle_recorder_panel.cpp include/panels/obstacle_recorder_panel.h
                                src/panels/obstacle_publisher_panel.cpp include/panels/obstacle_publisher_panel.h)
target_link_libraries(${PROJECT_NAME}_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_gui ${catkin_EXPORTED_TARGETS})

file(MAKE_DIRECTORY /home/$ENV{USER}/ObstacleDetector/records)
file(MAKE_DIRECTORY /home/$ENV{USER}/ObstacleDetector/resources)
file(COPY resources/play.png resources/stop.png DESTINATION /home/$ENV{USER}/ObstacleDetector/resources)
