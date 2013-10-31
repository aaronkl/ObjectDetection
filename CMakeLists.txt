cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
 
project(object_recognition)
  

set(PCL_DIR "/home/kleinaa/devel/pcl-trunk/build/")

find_package(PCL 1.7 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(HDF5)

find_package(FLANN)
include_directories(${FLANN_INCLUDE_DIRS})

include_directories(${HDF5_INCLUDE_DIR})

 
add_executable (object_recognition object_recognition.cpp main.cpp)
target_link_libraries (object_recognition ${PCL_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})