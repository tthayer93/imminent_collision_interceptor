cmake_minimum_required(VERSION 2.8.3)
project(imminent_collision_interceptor)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs tf)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(imminent_collision_interceptor src/imminent_collision_interceptor.cpp)
target_link_libraries(imminent_collision_interceptor ${catkin_LIBRARIES})
#add_dependencies(imminent_collision_interceptor)

