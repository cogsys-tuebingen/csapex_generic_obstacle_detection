cmake_minimum_required(VERSION 2.8.3)
project(generic_obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
    csapex
    csapex_point_cloud
    csapex_transform
    csapex_ros
    tf
    )

find_package(PCL 1.2 REQUIRED COMPONENTS common filters segmentation)

find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_definitions(-DQT_NO_KEYWORDS -march=native -O3)

catkin_package(
    INCLUDE_DIRS include
    DEPENDS Eigen3
    )

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )

file(GLOB_RECURSE HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h*)

add_library(${PROJECT_NAME}_apex_node
    ${HEADERS}

  src/obstacle_detection/low_pass_filter.cpp
  src/obstacle_detection/heat_calculator.cpp
  src/obstacle_detection/obstacle_detector.cpp
  src/obstacle_detection/detector.cpp
  src/obstacle_detection/histogram.cpp
  src/obstacle_detection/point_histogram.hpp
    )

target_link_libraries(${PROJECT_NAME}_apex_node
    ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES}
    )


install(FILES plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_apex_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
