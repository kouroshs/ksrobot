project(ksrobot)
cmake_minimum_required(VERSION 2.8)

# Set CMake build type here. the default is Debug.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "DEBUG")
  #set(CMAKE_BUILD_TYPE "RELEASE")
  #set(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
  #set(CMAKE_BUILD_TYPE "MINSIZEREL")
endif()

if( ${CMAKE_BUILD_TYPE} EQUAL "DEBUG" )
    add_definitions("-rdynamic")
endif()


#NOTE: This configuration should be set for each computer:
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN REQUIRED eigen3)

SET(QT_USE_QTOPENGL TRUE)
SET(QT_USE_QTNETWORK TRUE)

find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)
find_package(Boost COMPONENTS system filesystem thread date_time program_options chrono python REQUIRED)
find_package(PythonLibs 2.7.4 REQUIRED)
#TODO: For my system there is a bug which causes cmake to include python 2.7 include filesystem
#      but link to 3.2 libraries. Should be tested for each system and the following lines should be
#      removed and the above find_package should be un-commented.
set(PYTHON_LIBRARIES "/usr/lib/python2.7/config-x86_64-linux-gnu/libpython2.7.so")
set(PYTHON_INCLUDE_DIRS "/usr/include/python2.7/")


find_package(OpenNI REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(VTK REQUIRED)
find_package(octomap REQUIRED)
find_package(TBB REQUIRED)

#message(STATUS "==========================================================================")
#message(STATUS ${PYTHON_LIBRARIES})

# Use thread safe boost components
set(Boost_USE_MULTITHREADED ON)

set(GLOBAL_LIBRARIES ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${TBB_LIBRARIES})

include(${VTK_USE_FILE})


include_directories(${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR} 
                    ${Boost_INCLUDE_DIR}
                    ${EIGEN_INCLUDE_DIRS} ${OpenNI_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
                    
include_directories(./)


#Set compiler flags, currently only supported for GNU GCC
if(CMAKE_COMPILER_IS_GNUCXX)
    if(${CMAKE_BUILD_TYPE} EQUAL "DEBUG")
        add_definitions("-Wall -Wextra -Wno-reorder -Wno-enum-compare")
    else()
        add_definitions("-march=native -Wall -Wextra -Wno-reorder -Wno-enum-compare")
    endif(${CMAKE_BUILD_TYPE} EQUAL "DEBUG")
endif(CMAKE_COMPILER_IS_GNUCXX)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")

add_subdirectory(common)
add_subdirectory(interfaces)
add_subdirectory(gui)
add_subdirectory(libksrobot)
add_subdirectory(roboctrl)
add_subdirectory(apps)
