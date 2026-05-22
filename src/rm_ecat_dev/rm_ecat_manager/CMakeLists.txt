cmake_minimum_required(VERSION 3.5.0)
project(rm_ecat_manager)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall -Werror)
add_definitions(-DMELO_USE_COUT)

find_package(yaml-cpp REQUIRED)

set(CATKIN_PACKAGE_DEPENDENCIES
        rm_ecat_standard_slave
        rm_ecat_mit
        ecat_manager
        )


find_package(catkin REQUIRED
        COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/RmEcatStandardSlaveManager.cpp
        src/RmEcatMitManager.cpp
        )

target_link_libraries(
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
