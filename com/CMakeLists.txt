set (CMAKE_CXX_STANDARD 11)
cmake_minimum_required(VERSION 2.8)
project( ComAnalysis )
find_package( OpenCV REQUIRED )
add_executable( ComAnalysis ComAnalysis.cpp )
target_link_libraries( ComAnalysis ${OpenCV_LIBS} )

