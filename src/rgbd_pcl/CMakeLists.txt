cmake_minimum_required(VERSION 3.0.2)
project(rgbd_pcl)

set(OpenCV_DIR "/home/hiling/opencv_4_8_1/build/")


find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	sensor_msgs
	std_msgs
	image_transport
	cv_bridge
	message_filters
	message_generation
	tf2
	tf2_ros
	tf2_geometry_msgs
#	pcl_ros
#	pcl_conversions
	dynamic_reconfigure
	nodelet
	pluginlib
)
#
#find_package(PCL REQUIRED COMPONENTS common io visualization PassThrough)

find_package(
	OpenCV
)
#find_package(PCL REQUIRED COMPONENTS common io visualization PassThrough)

generate_dynamic_reconfigure_options(
#	cfg/param.cfg
	cfg/SideBar.cfg
)

add_service_files(
	FILES
	RequestInfo.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rgbd_pcl
#  CATKIN_DEPENDS opencv roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)





###########
## Build ##
###########

include_directories(
# include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIR}
	./include
#	${PCL_INCLUDE_DIRS}
)

FILE(GLOB SOURCE_FILES "./src/*.cpp")

#add_executable(
#	rgbd_pcl_exe src/main.cpp
#)

#add_library(
#	VisionExchangerNodelet
#	src/SideBar.cpp
#	src/FrontBar.cpp
#	src/AssemblyPost.cpp
#	src/nodelet_export.cpp
#	src/record_pointcloud.cpp
#
#	include/VisionExchanger_Base.h
#		include/AssemblyPost.h
#		include/SideBar.h
#		include/FrontBar.h
#
#)

#add_executable(
#	record_pointcloud src/record_pointcloud.cpp
#)

#add_dependencies(rgbd_pcl_exe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_dependencies(VisionExchangerNodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#target_link_libraries(rgbd_pcl_exe ${OpenCV_LIBRARIES})
#target_link_libraries(rgbd_pcl_exe ${catkin_LIBRARIES})
#target_link_libraries(rgbd_pcl_exe ${PCL_LIBRARIES})


#target_link_libraries(VisionExchangerNodelet ${OpenCV_LIBRARIES})
#target_link_libraries(VisionExchangerNodelet ${catkin_LIBRARIES})

#target_link_libraries(record_pointcloud ${OpenCV_LIBRARIES})
#target_link_libraries(record_pointcloud ${catkin_LIBRARIES})


# 安装 nodelet 库与插件描述文件，供 ROS 工具链发现
#install(TARGETS VisionExchangerNodelet
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(FILES vision_exchanger.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
