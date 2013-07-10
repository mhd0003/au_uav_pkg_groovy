cmake_minimum_required(VERSION 2.8.3)
project(au_uav_gui)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp roslib)
find_package(Qt4 REQUIRED COMPONENTS QtGui QtWebkit QtNetwork)
include(${QT_USE_FILE})

include_directories(/usr/local/Trolltech/Qt-4.8.4/include/)
include_directories(/usr/include/qt4/QtCore)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(
  CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake
)


find_package(GLIB2 REQUIRED)
find_package(GTHREAD2 REQUIRED)

add_message_files(FILES
  Telemetry.msg
)

add_service_files(FILES
  SendFilePath.srv
)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/au_uav_gui/*.hpp)
file(GLOB_RECURSE HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/au_uav_gui/*.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

generate_messages(DEPENDENCIES
  std_msgs
)

add_executable(au_uav_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${QT_MOC} ${HEADERS})
target_link_libraries(au_uav_gui ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(au_uav_gui ${PROJECT_NAME}_gencpp)

INCLUDE_DIRECTORIES(
  ${MAVLINK_INCLUDE_DIR}
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
  ../include/
)

include_directories(include ${BOOST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS roscpp roslib message_runtime
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)
