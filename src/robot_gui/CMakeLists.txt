cmake_minimum_required(VERSION 3.0.2)
project(robot_gui)


################################
##  CONFIGURE THIS FILE HERE  ##
################################

set(ROS_DEPENDS
  roscpp
  std_msgs
  rviz
  robot_navigation
  )

set(EXTERNAL_DEPENDS
  Qt5Widgets
  Qt5PrintSupport
  #OpenCV
  )

set(EXTERNAL_DEPENDS_LIB
  Qt5::Widgets
  Qt5::PrintSupport
  Qt5::Core
  #${OpenCV_LIBS}
  )

set(INCLUDE_DIRECTORIES
  #${OpenCV_LIBS}
  )


##########################################
##  Be Carefull editing below this line ##
##########################################

## Compile as C++17
add_compile_options(-std=c++17 -Wno-register)

## need this for qt auto generated headers to include!
set(CMAKE_INCLUDE_CURRENT_DIR ON)

###
###  Add Non ROS dependencies
###

foreach(DEPEND ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

###
###  Add any required ros packages here
###
find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDS})
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${ROS_DEPENDS}
  DEPENDS
    ${EXTERNAL_DEPENDS}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIRECTORIES}
  ${OpenCV_INCLUDE_DIRS}
)

###########
## Build ##
###########

file(GLOB HDRS
  "include/${PROJECT_NAME}/*.h"
)
file(GLOB SRCS
  "src/*.cpp"
  "src/*.hpp"
)
file(GLOB NODES
  "nodes/*.cpp"
)

###
### QT stuff (required if you want a GUI)
###
set(CMAKE_AUTOMOC ON)  #This will autogenerate qt gui stuff
set(CMAKE_AUTORCC ON)  #this needs to be on so cmake will know what to do with QT resource Files

#find_package(Qt5Widgets)
#find_package(Qt5PrintSupport)
#find all the qt UI stuff
file(GLOB_RECURSE UI
    "include/${PROJECT_NAME}/*.ui"
)

message("Adding ${PROJECT_NAME} UI files: " )
foreach(source ${UI})
    message("  - " ${source})
endforeach(source)

#make them into headers
qt5_wrap_ui (UI_HDRS  ${UI})

message("Generating ${PROJECT_NAME} UI Headers: " )
foreach(source ${UI_HDRS})
    message("  - " ${source})
endforeach(source)

# find all resource files
file(GLOB QRC
    "resource/*.qrc"
)

###
### compile
###

message("buliding ${PROJECT_NAME} node: ")
foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)
  message("  - " ${NODE_NAME})

  add_executable(${NODE_NAME} ${NODE_FILE} ${HDRS} ${SRCS} ${UI} ${QRC})
  add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries (${NODE_NAME} ${catkin_LIBRARIES}  ${EXTERNAL_DEPENDS_LIB} ${OpenCV_LIBS})
  target_include_directories(${NODE_NAME} PRIVATE "include/${PROJECT_NAME}/")

  list(APPEND NODE_LIST ${NODE_NAME})

endforeach(NODE_FILE)


install(TARGETS ${NODE_LIST}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
