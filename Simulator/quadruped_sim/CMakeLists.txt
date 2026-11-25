cmake_minimum_required(VERSION 3.10)
project(prometheus_quadruped_sim)

find_package(catkin REQUIRED COMPONENTS costmap_2d geometry_msgs nav_core nav_msgs pluginlib
  roscpp sensor_msgs std_msgs tf2_geometry_msgs tf2_ros)
find_package(gazebo REQUIRED)
catkin_package()

add_library(quadruped_proxy SHARED src/proxy.cpp)
target_compile_features(quadruped_proxy PRIVATE cxx_std_14)
target_include_directories(quadruped_proxy PRIVATE ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(quadruped_proxy PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(quadruped_physical SHARED src/physical.cpp)
target_compile_features(quadruped_physical PRIVATE cxx_std_17)
target_include_directories(quadruped_physical PRIVATE ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(quadruped_physical PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
catkin_python_setup()

add_library(quadruped_gait_local_planner SHARED src/gait_local_planner.cpp)
target_compile_features(quadruped_gait_local_planner PRIVATE cxx_std_17)
target_include_directories(quadruped_gait_local_planner PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(quadruped_gait_local_planner PRIVATE ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/bringup.py scripts/velocity_guard.py
  scripts/send_goal.py scripts/crossing.py test/m1_check.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS quadruped_proxy quadruped_physical quadruped_gait_local_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES local_planner_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch config models worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
