set(KINECTSAVE_SRCS main.cpp)

add_executable(ksrobot_kinectsave ${KINECTSAVE_SRCS})
target_link_libraries(ksrobot_kinectsave ksrobot_common ksrobot_interfaces ${QT_QTCORE_LIBRARY} 
    ${QT_QTGUI_LIBRARY})