set(KINECT_SRCS KinectDeviceReader.cpp KinectInterface.cpp KinectDatasetReader.cpp)

add_library(kinect STATIC  ${KINECT_SRCS})
target_link_libraries(kinect ${OpenCV_LIBS} ${OpenNI_LIBRARIES} ${GLOBAL_LIBRARIES})