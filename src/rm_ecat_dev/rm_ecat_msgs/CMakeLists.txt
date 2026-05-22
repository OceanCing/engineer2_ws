cmake_minimum_required(VERSION 3.10)
project(rm_ecat_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        geometry_msgs

        sensor_msgs
        )

add_message_files(
        FILES
        RmEcatStandardSlaveReading.msg
        RmEcatStandardSlaveReadings.msg
        RmEcatMitSlaveReading.msg
        RmEcatMitSlaveReadings.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        geometry_msgs
        sensor_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        geometry_msgs
        sensor_msgs
)
