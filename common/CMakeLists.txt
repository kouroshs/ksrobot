include_directories(../)

set(COMMON_SRC  RobotInfo.cpp OccupancyMap.cpp LoopDetectorInterface.cpp KinectImage.cpp 
                EngineInterface.cpp Timer.cpp Interface.cpp VisualOdometryInterface.cpp
                SLAMInterface.cpp MappingInterface.cpp MotionPlanner.cpp 
                ExecCtrlData.cpp SettingsBinder.cpp Worker.cpp ProgramOptions.cpp
                KinectInterface.cpp Defenitions.cpp)

find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})

add_library(ksrobot_common SHARED ${COMMON_SRC})
target_link_libraries(ksrobot_common ${GLOBAL_LIBRARIES} ${PNG_LIBRARY} octomap gtsam)

