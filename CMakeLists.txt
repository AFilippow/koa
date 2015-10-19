cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)




find_package(VTK REQUIRED)

include_directories(${VTK_INCLUDE_DIRS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()




SET(detector_SOURCES src/main.cpp  src/segmenter.cpp src/vrepComm.cpp src/xdmp.cpp src/vectormath.cpp src/cspaceconverter.cpp src/KukaLWR_DHnew.cpp) 
SET(detector_HEADERS src/segmenter.h src/vrepComm.h src/xdmp.h src/vectormath.h src/cspaceconverter.h src/KukaLWR_DHnew.h) 

rosbuild_add_executable(main ${detector_SOURCES} ${detector_HEADERS})
rosbuild_add_boost_directories()
rosbuild_link_boost(main signals)
  
target_link_libraries(main ${QT_QTCORE_LIBRARY} ${OROCOS_KDL} ${QT_QTGUI_LIBRARY} ${QT_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${QT_QTXML_LIBRARY} ${VTK_LIBRARIES} armadillo lapack blas)
 
 
 

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
