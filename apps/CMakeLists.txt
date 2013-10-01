#add_subdirectory(viewer)
add_subdirectory(component_test)
add_subdirectory(robocontrol)
add_subdirectory(kinect_capture)

file(GLOB PYTHON_FILES *.py)
file(COPY ${PYTHON_FILES} DESTINATION ../bin)
