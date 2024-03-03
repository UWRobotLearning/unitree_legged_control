cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    unitree_legged_msgs
)

catkin_package()

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-O3")

if( $ENV{UNITREE_SDK_VERSION} STREQUAL "3_1")
    include_directories($ENV{ALIENGO_SDK_PATH}/include)
    link_directories($ENV{ALIENGO_SDK_PATH}/lib)
    string(CONCAT LEGGED_SDK_NAME libaliengo_comm.so)
    set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

    add_definitions(-DSDK3_1)

    add_executable(lcm_server_3_1 $ENV{ALIENGO_SDK_PATH}/examples/lcm_server.cpp)
    target_link_libraries(lcm_server_3_1 ${EXTRA_LIBS} ${catkin_LIBRARIES})
    add_dependencies(lcm_server_3_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

elseif( $ENV{UNITREE_SDK_VERSION} STREQUAL "3_2")
    include_directories($ENV{UNITREE_LEGGED_SDK_PATH}/include)
    link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
    string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_$ENV{UNITREE_PLATFORM}.so)
    set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

    add_definitions(-DSDK3_2)

    add_executable(lcm_server_3_2 $ENV{UNITREE_LEGGED_SDK_PATH}/examples/lcm_server.cpp)
    target_link_libraries(lcm_server_3_2 ${EXTRA_LIBS} ${catkin_LIBRARIES})
    add_dependencies(lcm_server_3_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()

# add_executable(position_lcm src/exe/position_mode.cpp)
# target_link_libraries(position_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(position_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(velocity_lcm src/exe/velocity_mode.cpp)
# target_link_libraries(velocity_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(velocity_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(torque_lcm src/exe/torque_mode.cpp)
# target_link_libraries(torque_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(torque_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(walk_lcm src/exe/walk_mode.cpp)
# target_link_libraries(walk_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(walk_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(state_sub src/exe/state_sub.cpp)
target_link_libraries(state_sub ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(state_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_udp src/exe/ros_udp.cpp)
target_link_libraries(ros_udp ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_udp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_udp_high src/exe/ros_udp_high.cpp)
target_link_libraries(ros_udp_high ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_udp_high ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_udp_walk src/exe/ros_udp_walk.cpp)
target_link_libraries(ros_udp_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_udp_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(control_via_keyboard src/exe/control_via_keyboard.cpp)
target_link_libraries(control_via_keyboard ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(control_via_keyboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# cmake_minimum_required(VERSION 2.8.3)
# project(unitree_legged_control)

# set(CMAKE_CXX_FLAGS "-O3 -fPIC -std=c++11")

# find_package(catkin REQUIRED COMPONENTS
#     roscpp
#     tf2_ros
#     geometry_msgs
#     unitree_legged_msgs
#     unitree_legged_sdk
# )

# catkin_package()

# message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
# if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
#   set(ARCH amd64)
# else()
#   set(ARCH arm64)
# endif()

# set(EXTRA_LIBS -pthread ${unitree_legged_sdk_LIBRARIES})


# include_directories(
#     include
#     ${catkin_INCLUDE_DIRS}
#     ${unitree_legged_sdk_INCLUDE_DIRS}
# )

# # add_executable(ros_example_walk src/exe/ros_walk_example.cpp)
# # target_link_libraries(ros_example_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
# # add_dependencies(ros_example_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # add_executable(ros_example_position src/exe/ros_position_example.cpp)
# # target_link_libraries(ros_example_position ${EXTRA_LIBS} ${catkin_LIBRARIES})
# # add_dependencies(ros_example_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(state_sub src/exe/state_sub.cpp)
# target_link_libraries(state_sub ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(state_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(ros_udp src/exe/ros_udp.cpp)
# target_link_libraries(ros_udp ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(ros_udp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(ros_udp_high src/exe/ros_udp_high.cpp)
# target_link_libraries(ros_udp_high ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(ros_udp_high ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(ros_udp_walk src/exe/ros_udp_walk.cpp)
# target_link_libraries(ros_udp_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(ros_udp_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(control_via_keyboard src/exe/control_via_keyboard.cpp)
# target_link_libraries(control_via_keyboard ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(control_via_keyboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # add_executable(twist_sub src/exe/twist_sub.cpp)
# # target_link_libraries(twist_sub ${EXTRA_LIBS} ${catkin_LIBRARIES})
# # add_dependencies(twist_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # add_executable(ros_transform src/exe/ros_transform.cpp)
# # target_link_libraries(ros_transform ${EXTRA_LIBS} ${catkin_LIBRARIES})
# # add_dependencies(ros_transform ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

