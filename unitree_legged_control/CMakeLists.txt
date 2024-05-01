cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    image_transport
    sensor_msgs
    geometry_msgs
    vesc_msgs
    unitree_legged_msgs
    mavros_msgs
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

elseif($ENV{UNITREE_SDK_VERSION} STREQUAL "3_3_3")
    include_directories($ENV{UNITREE_LEGGED_SDK_PATH}/include)
    link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
    string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_$ENV{UNITREE_PLATFORM}.so)
    set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

    add_definitions(-DSDK3_3_3)

    add_executable(lcm_server_high_3_3_3 $ENV{UNITREE_LEGGED_SDK_PATH}/examples/lcm_server_high.cpp)
    target_link_libraries(lcm_server_high_3_3_3 ${EXTRA_LIBS} ${catkin_LIBRARIES})
    add_dependencies(lcm_server_high_3_3_3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    add_executable(lcm_server_low_3_3_3 $ENV{UNITREE_LEGGED_SDK_PATH}/examples/lcm_server_low.cpp)
    target_link_libraries(lcm_server_low_3_3_3 ${EXTRA_LIBS} ${catkin_LIBRARIES})
    add_dependencies(lcm_server_low_3_3_3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()

add_executable(state_sub src/exe/state_sub.cpp)
target_link_libraries(state_sub ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(state_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_udp src/exe/ros_udp.cpp)
target_link_libraries(ros_udp ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_udp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_udp_high src/exe/ros_udp_high.cpp)
target_link_libraries(ros_udp_high ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_udp_high ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(ros_udp_walk src/exe/ros_udp_walk.cpp)
# target_link_libraries(ros_udp_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(ros_udp_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(control_via_keyboard src/exe/control_via_keyboard.cpp)
target_link_libraries(control_via_keyboard ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(control_via_keyboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(A1_control_high src/exe/A1_control_high.cpp)
target_link_libraries(A1_control_high ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(A1_control_high ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(A1_reset src/exe/A1_reset.cpp)
target_link_libraries(A1_reset ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(A1_reset ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(position_control_via_H12 src/exe/position_control_via_H12.cpp)
target_link_libraries(position_control_via_H12 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(position_control_via_H12 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})