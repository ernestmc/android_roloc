cmake_minimum_required(VERSION 2.8.3)
project(usb_serial_for_android)

find_package(catkin REQUIRED rosjava_build_tools)
catkin_rosjava_setup(assembleRelease uploadArchives)
catkin_package()
