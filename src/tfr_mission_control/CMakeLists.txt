cmake_minimum_required(VERSION 2.8.3)

project(tfr_mission_control)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    rqt_gui
    rqt_gui_cpp
    tfr_msgs
    tfr_utilities
    actionlib
    roscpp
)

find_package(Qt5Widgets REQUIRED)

# Flags
SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(${PROJECT_NAME}_SRCS
    src/${PROJECT_NAME}/mission_control.cpp
)

set(${PROJECT_NAME}_HDRS
    include/${PROJECT_NAME}/mission_control.h
)

set(${PROJECT_NAME}_UIS
    src/${PROJECT_NAME}/mission_control.ui
)

set(${PROJECT_NAME}_QRC
    resource/resources.qrc
)

set(ui_INCLUDE_DIR
    "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
    include
    ${ui_INCLUDE_DIR}/..
)

if(NOT EXISTS ${ui_INCLUDE_DIR})
    file(MAKE_DIRECTORY ${ui_INCLUDE_DIR})
endif()

catkin_package(
    #  INCLUDE_DIRS
    #    ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
    #  LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    rqt_gui
    rqt_gui_cpp
    tfr_msgs
    tfr_utilities
    roscpp

)

# include directories before wrap cpp
include_directories(${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
    ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
    ${catkin_INCLUDE_DIRS}
)

include_directories(${Qt5Widgets_INCLUDE_DIRS})
add_definitions(${Qt5Widgets_DEFINITIONS})
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
qt5_add_resources(${PROJECT_NAME}_RCC ${${PROJECT_NAME}_QRC})

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${ui_INCLUDE_DIR})
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

add_library(${PROJECT_NAME}
    ${${PROJECT_NAME}_SRCS}
    ${${PROJECT_NAME}_HDRS}
    ${${PROJECT_NAME}_MOCS}
    ${${PROJECT_NAME}_UIS_H}
    ${${PROJECT_NAME}_RCC}
)

target_link_libraries(${PROJECT_NAME}
    status_code
    ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

add_executable(mission_control_subscriber ./src/mission_control_subscriber.cpp)
add_dependencies(mission_control_subscriber ${catkin_EXPORTED_TARGETS})
target_link_libraries(mission_control_subscriber ${catkin_LIBRARIES})


install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#install(
#  DIRECTORY resource
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  FILES_MATCHING
#  PATTERN "*.png"
#  PATTERN "*.svg"
#)
