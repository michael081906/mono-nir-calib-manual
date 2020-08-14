cmake_minimum_required(VERSION 2.8.3)
project(mono_nir_calib_manual)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  dynamic_reconfigure
  message_generation
)



 generate_dynamic_reconfigure_options(
   cfg/mono_nir_calib_cfg.cfg
 )

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/mono_nir_publish.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_node   ${catkin_LIBRARIES} )
