include_directories(../)

set(UTILS_SRC ExecCtrlData.cpp SettingsBinder.cpp Worker.cpp ThreadContainer.cpp Engine.cpp ProgramOptions.cpp)
add_subdirectory(kinect)

add_library(common STATIC ${UTILS_SRC})
target_link_libraries(common kinect ${GLOBAL_LIBRARIES} ${OpenCV_LIBS})

