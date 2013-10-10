include_directories(../)

set(COMMON_SRC  Defenitions.cpp 
                ProgramOptions.cpp
                Interface.cpp 
                VisualOdometryInterface.cpp
                KinectInterface.cpp
                RobotInfo.cpp 
                OccupancyMap.cpp 
                LoopDetectorInterface.cpp
                KinectImage.cpp 
                EngineInterface.cpp
                Timer.cpp 
                SLAMInterface.cpp 
                MappingInterface.cpp 
                MotionPlanner.cpp 
                ExecCtrlData.cpp 
                SettingsBinder.cpp 
                Worker.cpp 
                PCLUtils.cpp
                )

set(COMMON_PYTHON_SRCS  py/Interface_py.cpp py/ExecCtrlData_py.cpp
                        py/MappingInterface_py.cpp py/EngineInterface_py.cpp
                        py/KinectImage_py.cpp py/LoopDetectorInterface_py.cpp
                        py/KinectInterface_py.cpp py/MotionPlanner_py.cpp 
                        py/OccupancyMap_py.cpp py/ProgramOptions_py.cpp
                        py/RobotInfo_py.cpp py/SLAMInterface_py.cpp
                        py/Timer_py.cpp py/VisualOdometryInterface_py.cpp
                        )
                
find_package(PNG REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS} ${PNG_INCLUDE_DIR})

add_library(ksrobot_common SHARED ${COMMON_SRC} ${COMMON_PYTHON_SRCS})
target_link_libraries(ksrobot_common ${PYTHON_LIBRARIES} ${GLOBAL_LIBRARIES} ${PNG_LIBRARY} octomap gtsam)
cotire(ksrobot_common)
