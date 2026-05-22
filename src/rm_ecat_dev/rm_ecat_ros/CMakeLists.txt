cmake_minimum_required(VERSION 3.5.0)
project(rm_ecat_ros)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall -Werror)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        rm_ecat_manager
        rm_ecat_msgs
        any_node
        controller_manager
        urdf
        transmission_interface
        joint_limits_interface
        rm_common
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        rm_ecat_manager
        rm_ecat_msgs
        any_node
        controller_manager
        urdf
        transmission_interface
        joint_limits_interface
        rm_common
)

# needed to circumvent LD_LIBRARY_PATH being emptied through ethercat_grant
# in addition to not propagating rm_ecat_hw RUNPATH to dependencies, in contrast to RPATH
SET(GCC_NEWDTAGS_LINK_FLAGS "-Wl,--disable-new-dtags")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_NEWDTAGS_LINK_FLAGS}")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/RosMsgConversions.cpp
        src/RmEcatHardwareInterface.cpp
        src/RmStandardSlaveManagerRos.cpp
        src/RmMitSlaveManagerRos.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

add_executable(rm_ecat_hw
        src/RmEcatHardwareInterface.cpp
        src/RmEcatHardwareInterfaceNode.cpp
        )

target_link_libraries(rm_ecat_hw
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

install(TARGETS ${PROJECT_NAME} rm_ecat_hw
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
