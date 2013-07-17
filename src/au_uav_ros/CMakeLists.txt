cmake_minimum_required(VERSION 2.8.3)
project(au_uav_ros)
find_package(catkin REQUIRED
  COMPONENTS message_generation std_msgs roscpp rospy roslib
)

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_message_files(FILES
  Command.msg
  Telemetry.msg
)

add_service_files(FILES
  Ack.srv
  AddPlane.srv
  Centralize.srv
  CloseComms.srv
  Double.srv
  ExitCommand.srv
  InitComms.srv
  LoadCourse.srv
  Plane.srv
  RemovePlane.srv
  RemoveWp.srv
  SendFilePath.srv
  SetWp.srv
  SimPlane.srv
  Wp.srv
)

SET(
  CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake
)

include_directories(include ${BOOST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

find_package(GLIB2 REQUIRED)
find_Package(GTHREAD2 REQUIRED)


INCLUDE_DIRECTORIES(
  ${MAVLINK_INCLUDE_DIR}
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
  ../include/
)

generate_messages(DEPENDENCIES
  std_msgs
)

add_library(standardDefs src/standardDefs.cpp include/au_uav_ros/standardDefs.h)
add_library(standardFuncs src/standardFuncs.cpp include/au_uav_ros/standardFuncs.h)
add_library(planeBuilder src/planeBuilder.cpp include/au_uav_ros/planeBuilder.h)
add_library(planeObject src/planeObject.cpp include/au_uav_ros/planeObject.h)
add_library(simPlaneObject src/simPlaneObject.cpp include/au_uav_ros/simPlaneObject.h)
add_library(vmath src/vmath.cpp include/au_uav_ros/vmath.h)
add_library(ripna src/ripna.cpp include/au_uav_ros/ripna.h)
add_library(vector2D src/vector2D.cpp include/au_uav_ros/vector2D.h)
add_library(ipn src/ipn.cpp include/au_uav_ros/ipn.h)
add_library(collisionAvoidance src/collisionAvoidance.cpp include/au_uav_ros/collisionAvoidance.h)

add_executable(simulator src/simulator.cpp include/au_uav_ros/simulator.h)
add_executable(xbeeOut src/xbeeOut.cpp include/au_uav_ros/xbeeOut.h)
add_executable(xbeeIn src/xbeeIn.cpp include/au_uav_ros/xbeeIn.h)
add_executable(guiInterfacer src/guiInterfacer.cpp include/au_uav_ros/guiInterfacer.h)
add_executable(coordinator src/coordinator.cpp include/au_uav_ros/coordinator.h)
add_executable(evaluator src/evaluator.cpp include/au_uav_ros/evaluator.h)


add_dependencies(standardDefs ${PROJECT_NAME}_gencpp)
add_dependencies(standardFuncs ${PROJECT_NAME}_gencpp)
add_dependencies(simulator ${PROJECT_NAME}_gencpp)
add_dependencies(simPlaneObject ${PROJECT_NAME}_gencpp)
add_dependencies(xbeeIn ${PROJECT_NAME}_gencpp)
add_dependencies(xbeeOut ${PROJECT_NAME}_gencpp)
add_dependencies(guiInterfacer ${PROJECT_NAME}_gencpp)
add_dependencies(coordinator ${PROJECT_NAME}_gencpp)
add_dependencies(planeBuilder ${PROJECT_NAME}_gencpp)
add_dependencies(evaluator ${PROJECT_NAME}_gencpp)

target_link_libraries(xbeeIn
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
  ${catkin_LIBRARIES}
)

target_link_libraries(xbeeOut
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
  ${catkin_LIBRARIES}
)
#target_link_libraries(
#  ${PROJECT_NAME}
#  ${BOOST_LIBRARIES}
#)

target_link_libraries(ripna vmath standardFuncs)
target_link_libraries(ipn vector2D)
target_link_libraries(standardDefs ${BOOST_LIBRARIES})
target_link_libraries(planeBuilder standardDefs)
target_link_libraries(planeObject standardFuncs standardDefs)
target_link_libraries(collisionAvoidance ripna ipn)
target_link_libraries(simPlaneObject planeObject)
target_link_libraries(simulator ${catkin_LIBRARIES} simPlaneObject)
target_link_libraries(guiInterfacer ${catkin_LIBRARIES})
target_link_libraries(coordinator ${catkin_LIBRARIES} planeBuilder simPlaneObject collisionAvoidance)
target_link_libraries(evaluator ${catkin_LIBRARIES} standardDefs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp au_uav_gui message_runtime
)
