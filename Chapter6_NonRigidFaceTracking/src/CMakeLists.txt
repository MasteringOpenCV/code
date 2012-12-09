PROJECT(OPENCV_HOTSHOTS)

cmake_minimum_required(VERSION 2.8)
find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(
${OPENCV_INCLUDE_DIR}
${CMAKE_CURRENT_SOURCE_DIR}
)
SET( LIBRARY_OUTPUT_PATH ${OPENCV_HOTSHOTS_BINARY_DIR}/../lib CACHE PATH
     "Output directory for the OpenCV_hotshots library" )
SET( EXECUTABLE_OUTPUT_PATH 
     ${OPENCV_HOTSHOTS_BINARY_DIR}/../bin CACHE PATH
     "Output directory for the OpenCV_hotshots executables" )

SUBDIRS(opencv_hotshots/ft)
