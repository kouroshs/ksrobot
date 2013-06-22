include_directories(../../)
set(COMPONENT_TEST_SRCS main.cpp)
add_executable(component_test ${COMPONENT_TEST_SRCS})
target_link_libraries(component_test ksrobot_gui ksrobot_common ksrobot_interfaces)