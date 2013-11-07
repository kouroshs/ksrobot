set(KSROBOT_PYTHON_SRCS libksrobot.cpp libksrobot_interfaces.cpp libksrobot_common.cpp libksrobot_eigen.cpp libksrobot_pcl.cpp)

include_directories(${PYTHON_INCLUDE_DIRS})

add_library(ksrobot SHARED ${KSROBOT_PYTHON_SRCS})
target_link_libraries(ksrobot ${PYTHON_LIBRARIES} ${GLOBAL_LIBRARIES} ksrobot_common 
#                                                                       ksrobot_gui 
                                                                      ksrobot_interfaces)