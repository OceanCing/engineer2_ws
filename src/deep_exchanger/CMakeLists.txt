cmake_minimum_required(VERSION 3.0.2)
project(deep_exchanger)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        cv_bridge
        nodelet
        pluginlib
        dynamic_reconfigure
        tf
        rm_msgs
)

generate_dynamic_reconfigure_options(
        cfg/dynamic.cfg
)

find_package(OpenCV REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(ngraph REQUIRED)
find_package(CGAL REQUIRED)
include(${CGAL_USE_FILE})

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp dynamic_reconfigure std_msgs cv_bridge dynamic_reconfigure rm_msgs
        LIBRARIES ${PROJECT_NAME}
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
)


add_library(
        ${PROJECT_NAME}
        src/deep_exchanger_nodelet.cpp
        src/deep_exchanger_base.cpp
        src/lsd.cpp
)

add_dependencies(deep_exchanger ${PROJECT_NAME}_gencfg)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${InferenceEngine_LIBRARIES}
        ${NGRAPH_LIBRARIES}
        ${OpenCV_LIBS}
        )
