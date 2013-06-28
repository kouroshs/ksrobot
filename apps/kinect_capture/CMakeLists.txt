set(KINECT_CAPTURE_SRCS main.cpp)
add_executable(kinect_capture ${KINECT_CAPTURE_SRCS})
target_link_libraries(kinect_capture ksrobot_common ksrobot_interfaces ${GLOBAL_LIBRARIES})