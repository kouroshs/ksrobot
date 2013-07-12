include_directories(../)

set(COMMON_SRC LoopDetector.cpp KinectImage.cpp Timer.cpp Interface.cpp VisualOdometryInterface.cpp EngineInterface.cpp 
              SLAMInterface.cpp MappingInterface.cpp PlannerInterface.cpp 
              ExecCtrlData.cpp SettingsBinder.cpp Worker.cpp ProgramOptions.cpp
              KinectInterface.cpp Defenitions.cpp)

find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})

add_library(ksrobot_common SHARED ${COMMON_SRC})
target_link_libraries(ksrobot_common ${GLOBAL_LIBRARIES} ${PNG_LIBRARY} octomap gtsam)

