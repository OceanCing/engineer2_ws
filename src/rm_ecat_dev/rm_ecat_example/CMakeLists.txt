cmake_minimum_required(VERSION 3.5.0)
project(rm_ecat_example)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall -Werror)
add_definitions(-DMELO_USE_COUT)

find_package(catkin REQUIRED
        COMPONENTS
        rm_ecat_manager
        )

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS
        rm_ecat_manager
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(rm_ecat_standard_example
        src/RmEcatStandardExample.cpp
        )

add_executable(rm_ecat_mit_example
        src/RmEcatMitExample.cpp
        )

target_link_libraries(
        rm_ecat_standard_example
        ${catkin_LIBRARIES}
        -pthread
)

target_link_libraries(
        rm_ecat_mit_example
        ${catkin_LIBRARIES}
        -pthread
)

install(TARGETS rm_ecat_standard_example rm_ecat_mit_example
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
