cmake_minimum_required(VERSION 2.8.3)
project(camera_link_rev)

add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS message_generation roscpp geometry_msgs tf2_ros nav_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(camera_link_rev_node src/main.cpp src/camera_link_rev.cpp)

target_link_libraries(camera_link_rev_node ${catkin_LIBRARIES})

# add_dependencies(camera_link_rev_node )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS camera_link_rev_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
