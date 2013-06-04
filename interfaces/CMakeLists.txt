include_directories(../)

set(INTERFACES_SRCS FovisInterface.cpp KinectDatasetReader.cpp KinectDeviceReader.cpp MTEngine.cpp SerialEngine.cpp)

add_library(ksrobot_interfaces SHARED ${INTERFACES_SRCS})
target_link_libraries(ksrobot_interfaces ${GLOBAL_LIBRARIES} ksrobot_common)