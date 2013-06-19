cmake_minimum_required(VERSION 2.8.3)
project(au_uav_ros)
find_package(catkin REQUIRED
  COMPONENTS message_generation std_msgs roscpp rospy roslib
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_message_files(FILES
  Command.msg
  TelemetryUpdate.msg
)

add_service_files(FILES
  CreateSimulatedPlane.srv
  DeleteSimulatedPlane.srv
  GoToWaypoint.srv
  LoadCourse.srv
  LoadPath.srv
  RequestPlaneID.srv
  RequestWaypointInfo.srv
  SaveFlightData.srv
  SendFilePath.srv
  StartCollisionAvoidance.srv
  StartCoordinator.srv
  StartSimulator.srv
  StartXbeeIO.srv
)

SET(
  CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake
)

include_directories(include ${BOOST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

find_package(GLIB2 REQUIRED)
find_Package(GTHREAD2 REQUIRED)


generate_messages(DEPENDENCIES
  std_msgs
)

INCLUDE_DIRECTORIES(
  ${MAVLINK_INCLUDE_DIR}
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
  ../include/
)

add_executable(simulator src/simulator.cpp src/au_uav_ros/SimulatedPlane.cpp src/au_uav_ros/standardDefs.cpp src/au_uav_ros/WindSimulation.cpp src/au_uav_ros/GPSErrorSimulation.cpp src/au_uav_ros/Simulation.cpp)
add_executable(collisionAvoidance src/collisionAvoidance.cpp src/au_uav_ros/standardDefs.cpp src/standardFuncs.cpp src/planeObject.cpp src/ripna.cpp src/vmath.cpp)
add_executable(coordinator src/coordinator.cpp src/au_uav_ros/PlaneCoordinator.cpp src/au_uav_ros/standardDefs.cpp)
add_executable(ControlMenu src/ControlMenu.cpp src/au_uav_ros/standardDefs.cpp)
add_executable(KMLCreator src/KMLCreator.cpp)
add_executable(visualization src/visualization.cpp)
add_executable(evaluator src/evaluator.cpp src/au_uav_ros/standardDefs.cpp)
add_executable(XbeeIO src/XbeeIO.cpp)
add_executable(guiInterfacer src/guiInterfacer.cpp src/au_uav_ros/standardDefs.cpp)

add_dependencies(simulator ${PROJECT_NAME}_gencpp)
add_dependencies(collisionAvoidance ${PROJECT_NAME}_gencpp)
add_dependencies(coordinator ${PROJECT_NAME}_gencpp)
add_dependencies(KMLCreator ${PROJECT_NAME}_gencpp)
add_dependencies(visualization ${PROJECT_NAME}_gencpp)
add_dependencies(XbeeIO ${PROJECT_NAME}_gencpp)
add_dependencies(guiInterfacer ${PROJECT_NAME}_gencpp)
add_dependencies(ControlMenu ${PROJECT_NAME}_gencpp)

target_link_libraries(XbeeIO
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
)

#target_link_libraries(
#  ${PROJECT_NAME}
#  ${BOOST_LIBRARIES}
#)

target_link_libraries(simulator Geographic noise)
target_link_libraries(simulator ${catkin_LIBRARIES})
target_link_libraries(collisionAvoidance ${catkin_LIBRARIES})
target_link_libraries(coordinator ${catkin_LIBRARIES})
target_link_libraries(ControlMenu ${catkin_LIBRARIES})
target_link_libraries(KMLCreator ${catkin_LIBRARIES})
target_link_libraries(visualization ${catkin_LIBRARIES})
target_link_libraries(evaluator ${catkin_LIBRARIES})
target_link_libraries(XbeeIO ${catkin_LIBRARIES})
target_link_libraries(guiInterfacer ${catkin_LIBRARIES})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp au_uav_gui message_runtime
)
