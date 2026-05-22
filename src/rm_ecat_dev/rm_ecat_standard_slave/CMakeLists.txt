cmake_minimum_required(VERSION 3.5.0)
project(rm_ecat_standard_slave)

set(CMAKE_CXX_STANDARD 17)

add_definitions(-Wall)
add_definitions(-DMELO_USE_COUT)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
        soem_interface_rsl
        rm_msgs
        )

find_package(catkin REQUIRED
        COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        )

find_package(yaml-cpp REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

include_directories(
        include
        SYSTEM
        ${catkin_INCLUDE_DIRS}
)

file(GLOB SOURCES src/*.cpp)

add_library(${PROJECT_NAME} ${SOURCES}
        )

add_dependencies(
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
)

#############
## Install ##
#############

install(
        TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
        DIRECTORY
        include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
