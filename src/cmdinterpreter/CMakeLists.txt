cmake_minimum_required(VERSION 2.8.3)
project(cmdinterpreter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  roscpp
  roslib
  cmake_modules
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  tf
tf_conversions
eigen_conversions
rviz_visual_tools
)

find_package(Boost REQUIRED system thread chrono regex log_setup log locale )
find_package(Eigen REQUIRED)

#The catkin_package function specifies which dependencies your package exports to its dependencies. In particular, #INCLUDE_DIRS specifies which directory in your package contains the header directory that should be on the include #path for any packages that compile against your package.

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    
  CATKIN_DEPENDS
    roscpp
    roslib
    moveit_core
    sensor_msgs
    trajectory_msgs
    geometry_msgs
    tf
tf_conversions
eigen_conversions
rviz_visual_tools
  DEPENDS
    Boost
    Eigen


#  CFG_EXTRAS
#    descartes_core-extras.cmake
)

###########
## Build ##
###########
# set(CMAKE_CXX_COMPILER "g++")
#set(CMAKE_CXX_FLAGS "-Wdeprecated-declarations ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-w ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-DDEBUG ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-DBOOST_LOG_DYN_LINK ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-fpermissive ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}")

#set(CMAKE_CXX_FLAGS "-DCHECKERS ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "-DBOLTDEMO ${CMAKE_CXX_FLAGS}")


set(CMAKE_EXE_LINKER_FLAGS "-rdynamic ${CMAKE_EXE_LINKER_FLAGS}")

message("DEBUG variable catkin_INCLUDEDIR: ${catkin_INCLUDEDIR}")
message("DEBUG variable Eigen_INCLUDE_DIRS:  ${Eigen_INCLUDE_DIRS}")

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    # include breakdown for this project
    include/${PROJECT_NAME}
    include/${PROJECT_NAME}/NIST
   /usr/include/eigen3
)



add_executable(cmdinterpreter
src/Conversions.cpp
src/RvizMarker.cpp
src/main.cpp
)

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*.h)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})



link_directories(/usr/lib/x86_64-linux-gnu/)
link_directories(${CATKIN_DEVEL_PREFIX}/install})

target_link_libraries(cmdinterpreter
libtinyxml.a  
# I had to add ${catkin_LIBRARIES} AND remove the reference to the shared library 
${catkin_LIBRARIES}
${Boost_LIBRARIES}
)


#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    
    
    
    
