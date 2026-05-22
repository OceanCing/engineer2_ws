cmake_minimum_required(VERSION 3.5.0)
project(ecat_manager)

find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    message(STATUS "Using ccache")
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif ()

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall -Werror -Wextra)

set(CATKIN_PACKAGE_DEPENDENCIES
        soem_interface_rsl
        any_worker
        signal_handler
)

find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED
        COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        )

set(PACKAGE_DEPENDENCIES
        )

find_package(cleardrive_foc QUIET)
find_package(cleardrive_imu QUIET)
find_package(rm_ecat_mit QUIET)
find_package(rm_ecat_standard_slave QUIET)

if (cleardrive_foc_FOUND)
    add_definitions(-D_CLEAR_FOC_FOUND_)
    LIST(APPEND PACKAGE_DEPENDENCIES cleardrive_foc)
endif ()
if (cleardrive_imu_FOUND)
    add_definitions(-D_CLEAR_IMU_FOUND_)
    LIST(APPEND PACKAGE_DEPENDENCIES cleardrive_imu)
endif ()
if (rm_ecat_mit_FOUND)
    add_definitions(-D_MIT_FOUND_)
    LIST(APPEND PACKAGE_DEPENDENCIES rm_ecat_mit)
endif ()
if (rm_ecat_standard_slave_FOUND)
    add_definitions(-D_RM_FOUND_)
    LIST(APPEND PACKAGE_DEPENDENCIES rm_ecat_standard_slave)
endif ()

foreach (sdk ${PACKAGE_DEPENDENCIES})
    message("Found EtherCAT slave: " ${sdk})
endforeach ()

LIST(APPEND CATKIN_PACKAGE_DEPENDENCIES
        ${PACKAGE_DEPENDENCIES})

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
        ${cleardrive_foc_INCLUDE_DIRS}
        ${cleardrive_imu_INCLUDE_DIRS}
        ${rm_ecat_mit_INCLUDE_DIRS}
        ${rm_ecat_standard_slave_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/EcatBusManager.cpp
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
