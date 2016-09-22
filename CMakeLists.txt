cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detector)

set(CMAKE_CXX_FLAGS "-std=c++0x -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall") # or type -DCMAKE_BUILD_TYPE=Debug
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall") # or type -DCMAKE_BUILD_TYPE=Release

find_package(catkin REQUIRED COMPONENTS roscpp rviz std_msgs std_srvs geometry_msgs sensor_msgs visualization_msgs tf message_generation)
find_package(Armadillo REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

add_message_files(FILES CircleObstacle.msg SegmentObstacle.msg Obstacles.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
   INCLUDE_DIRS include /usr/include/OGRE
   LIBRARIES obstacle_detector
   CATKIN_DEPENDS roscpp rviz std_msgs std_srvs geometry_msgs sensor_msgs visualization_msgs tf message_runtime
   DEPENDS systemlib QtCore QtGui
)

qt4_wrap_cpp(MOC_FILES
  include/virtual_obstacle_publisher_panel.h
)

include_directories(include /usr/include/OGRE ${catkin_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(scans_merger src/scans_merger.cpp include/scans_merger.h include/math_utilities.h)
target_link_libraries(scans_merger ${catkin_LIBRARIES})
add_dependencies(scans_merger ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(obstacle_detector src/obstacle_detector.cpp include/obstacle_detector.h include/math_utilities.h
               include/figures/point.h include/figures/segment.h include/figures/circle.h include/figures/figure_fitting.h)
target_link_libraries(obstacle_detector ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})
add_dependencies(obstacle_detector ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(obstacle_tracker src/obstacle_tracker.cpp include/obstacle_tracker.h include/kalman.h include/tracked_obstacle.h)
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

add_executable(virtual_obstacle_publisher src/virtual_obstacle_publisher.cpp include/virtual_obstacle_publisher.h)
target_link_libraries(virtual_obstacle_publisher ${catkin_LIBRARIES})
add_dependencies(virtual_obstacle_publisher ${PROJECT_NAME}_gencpp)

add_library(${PROJECT_NAME}_gui src/virtual_obstacle_publisher_panel.cpp include/virtual_obstacle_publisher_panel.h ${MOC_FILES})
target_link_libraries(${PROJECT_NAME}_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_gui ${catkin_EXPORTED_TARGETS})
