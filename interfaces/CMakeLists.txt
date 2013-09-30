include_directories(../)

set(INTERFACES_SRCS LoopDetectorAVT.cpp RRTPlanner.cpp OctomapInterface.cpp ICPInterface.cpp iSAM2Interface.cpp FovisInterface.cpp KinectDatasetReader.cpp 
KinectDeviceReader.cpp MTEngine.cpp SerialEngine.cpp)

link_directories("${LIBPMK_PATH}/libpmk2/build ${LIBPMK_PATH}/avt/build")

add_library(ksrobot_interfaces SHARED ${INTERFACES_SRCS})
target_link_libraries(ksrobot_interfaces ${GLOBAL_LIBRARIES} ${OMPL_LIBRARIES} ksrobot_common fovis gtsam octomap)