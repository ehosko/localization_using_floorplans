cmake_minimum_required(VERSION 3.0.0)
project(localization_using_floorplans)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

catkin_package()

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        # Module
        src/localization/simple_localizer.cpp      
        src/localization/cloudGenerator/source_generator.cpp
        src/localization/cloudGenerator/target_generator.cpp
        )

############
# BINARIES #
############
cs_add_executable(ros_localization_node
        src/ros_localization_node.cpp
        )
target_link_libraries(ros_localization_node ${PROJECT_NAME})
include_directories(
  ${catkin_INCLUDE_DIRS}
)

cs_install()
cs_export()