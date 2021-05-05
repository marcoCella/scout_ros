cmake_minimum_required(VERSION 3.0.2)
project(project1)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation nav_msgs std_msgs message_filters dynamic_reconfigure tf2 tf2_ros robotics_hw1)

generate_dynamic_reconfigure_options(cfg/parameters.cfg)

add_service_files(FILES resetOdom.srv setOdom.srv)

add_message_files(FILES OdomInt.msg er_array.msg)

generate_messages(DEPENDENCIES std_msgs nav_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS std_msgs nav_msgs message_runtime message_filters)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(velpubsub src/velpubsub.cpp)
add_dependencies(velpubsub ${catkin_EXPORTED_TARGETS})
target_link_libraries(velpubsub ${catkin_LIBRARIES})

add_executable(odompubsub src/odompubsub.cpp)
add_dependencies(odompubsub ${PROJECT_NAME}_gencfg)
target_link_libraries(odompubsub ${catkin_LIBRARIES})

add_executable(residuals src/residuals.cpp)
target_link_libraries(residuals ${catkin_LIBRARIES})
add_dependencies(residuals ${catkin_EXPORTED_TARGETS})
