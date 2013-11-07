include_directories(../)

set(INTERFACES_SRCS LoopDetectorAVT.cpp 
                    RRTPlanner.cpp 
                    OctomapInterface.cpp 
                    ICPInterface.cpp 
                    iSAM2Interface.cpp 
                    FovisInterface.cpp 
                    KinectDatasetReader.cpp 
                    KinectDeviceReader.cpp 
#                    MTEngine.cpp 
#                    SerialEngine.cpp
                    )

set(INTERFACES_PY_SRCS LoopDetectorRTABMAP.cpp  py/iSAM2Interface_py.cpp
                        py/FovisInterface_py.cpp
                        py/KinectDatasetReader_py.cpp 
                        py/KinectDeviceReader_py.cpp 
                        py/OctomapInterface_py.cpp 
                        py/RRTPlanner_py.cpp
#                        py/MTEngine_py.cpp
                        )

include_directories(${PYTHON_INCLUDE_DIRS})

add_library(ksrobot_interfaces SHARED ${INTERFACES_SRCS} ${INTERFACES_PY_SRCS})
target_link_libraries(ksrobot_interfaces ${PYTHON_LIBRARIES} ${GLOBAL_LIBRARIES} ${OMPL_LIBRARIES} ksrobot_common fovis gtsam octomap)
#cotire(ksrobot_interfaces)
