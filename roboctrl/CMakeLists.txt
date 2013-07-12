include_directories(../)

set(ROBOCTRL_SRCS SerialPort.cpp RobXControl.cpp RobXControlDialog.cpp)
qt4_wrap_cpp(ROBOCTRL_MOCS RobXControl.h RobXControlDialog.h)
qt4_automoc(${ROBOCTRL_SRCS})

add_library(ksrobot_roboctrl SHARED ${ROBOCTRL_SRCS} ${ROBOCTRL_MOCS})
target_link_libraries(ksrobot_roboctrl ${GLOBAL_LIBRARIES} ksrobot_common)

