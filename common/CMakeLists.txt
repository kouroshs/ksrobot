include_directories(../)

set(UTILS_SRC Interface.cpp VisualOdometryInterface.cpp EngineInterface.cpp ExecCtrlData.cpp SettingsBinder.cpp Worker.cpp ProgramOptions.cpp)

add_library(ksrobot_common SHARED ${UTILS_SRC})
target_link_libraries(ksrobot_common ${GLOBAL_LIBRARIES})

