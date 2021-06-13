cmake_minimum_required(VERSION 3.0.2)
project(smb_powerstatus)

find_package(catkin REQUIRED COMPONENTS
    sensor_msgs 
    std_msgs
    message_generation
    roscpp
    rospy
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    SMBPowerStatus.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

