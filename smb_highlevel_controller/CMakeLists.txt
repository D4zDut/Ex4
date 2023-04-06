cmake_minimum_required(VERSION 3.0.2)
project(smb_highlevel_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  # LIBRARIES smb_highlevel_controller
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs
    geometry_msgs
  # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(smb_highlevel_controller
  src/smb_highlevel_controller_node.cpp
  src/SmbHighlevelController.cpp 
)

target_link_libraries(smb_highlevel_controller 
  ${catkin_LIBRARIES}
)






# add_executable(subscriber_scan src/subscriber_scan.cpp)
# target_link_libraries(subscriber_scan ${catkin_LIBRARIES})

# add_executable(subscriber_parameter src/subscriber_parameter.cpp)
# target_link_libraries(subscriber_parameter ${catkin_LIBRARIES})

# add_executable(subscriber_min_range src/subscriber_min_range.cpp)
# target_link_libraries(subscriber_min_range ${catkin_LIBRARIES})

# add_executable(pillar_position src/pillar_position.cpp)
# target_link_libraries(pillar_position ${catkin_LIBRARIES})



# add_executable(smb_pillar src/smb_pillar.cpp)
# target_link_libraries(smb_pillar ${catkin_LIBRARIES})
