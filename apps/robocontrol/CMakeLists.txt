set(ROBOCONTROL_SRCS main.cpp)
add_executable(ksrobot_robocontrol ${ROBOCONTROL_SRCS})
target_link_libraries(ksrobot_robocontrol ksrobot_common ksrobot_roboctrl ${QT_QTCORE_LIBRARY} 
                       ${QT_QTGUI_LIBRARY} ${QT_QTNETWORK_LIBRARY} ${PYTHON_LIBRARIES})
#cotire(ksrobot_robocontrol)