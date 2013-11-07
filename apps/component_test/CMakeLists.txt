include_directories(../../)
set(COMPONENT_TEST_SRCS main.cpp)
add_executable(component_test ${COMPONENT_TEST_SRCS})
target_link_libraries(component_test ksrobot_common ksrobot_interfaces ${GLOBAL_LIBRARIES} ${QT_QTCORE_LIBRARY} 
                       ${QT_QTGUI_LIBRARY} ${QT_QTNETWORK_LIBRARY} ${PYTHON_LIBRARIES})
#cotire(component_test)