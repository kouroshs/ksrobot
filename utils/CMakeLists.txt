include_directories(../)

set(UTILS_SRC Worker.cpp ThreadContainer.cpp Engine.cpp ProgramOptions.cpp)
add_subdirectory(kinect)

add_library(utils STATIC ${UTILS_SRC})
target_link_libraries(utils kinect ${GLOBAL_LIBRARIES} ${OpenCV_LIBS})

