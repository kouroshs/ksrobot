include_directories(../../)
set(VIEWER_SRC VisualizerApplication.cpp main.cpp)
add_executable(ksrobot_viewer ${VIEWER_SRC})

target_link_libraries(ksrobot_viewer ksrobot_gui ksrobot_common ksrobot_interfaces)
cotire(ksrobot_viewer)