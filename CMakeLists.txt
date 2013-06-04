project(ksrobot)
cmake_minimum_required(VERSION 2.8)

#NOTE: This configuration should be set for each computer:
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN REQUIRED eigen3)

SET(QT_USE_QTOPENGL TRUE)
SET(QT_USE_QTNETWORK TRUE)

find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)
find_package(Boost COMPONENTS system filesystem thread date_time program_options chrono REQUIRED)
find_package(OpenNI REQUIRED)

find_package(PCL 1.7 COMPONENTS common io REQUIRED)

#find_package(OpenCV REQUIRED)

# Use thread safe boost components
set(Boost_USE_MULTITHREADED ON)

set(GLOBAL_LIBRARIES ${PCL_LIBRARIES} ${Boost_LIBRARIES})

include_directories(${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR} 
                    ${Boost_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} 
                    ${EIGEN_INCLUDE_DIRS} ${OpenNI_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
                    
include_directories(./)


#Set compiler flags, currently only supported for GNU GCC
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wextra -Wno-reorder -Wno-enum-compare")
    #set(CMAKE_EXE_LINKER_FLAGS "-s")  ## Strip binary
endif()


add_subdirectory(common)
add_subdirectory(interfaces)
add_subdirectory(gui)
add_subdirectory(apps)
