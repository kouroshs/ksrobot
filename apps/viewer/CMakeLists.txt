
include_directories(../../)
set(VIEWER_SRC VisualizerApplication.cpp main.cpp)
add_executable(viewer ${VIEWER_SRC})

target_link_libraries(viewer gui utils ${OpenCV_LIBS})
