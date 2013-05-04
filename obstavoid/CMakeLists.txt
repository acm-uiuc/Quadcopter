cmake_minimum_required(VERSION 2.6)

project(obstavoid)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
#include_directories("/home/ndryden/include")
#link_directories("/home/ndryden/lib")

set(obstavoid_components mops
                         #anymore components
			 )

foreach(DIR ${obstavoid_components})
  set(obstavoid_src_dirs ${obstavoid_src_dirs}
                         ${PROJECT_SOURCE_DIR}/${DIR})
endforeach()

include_directories(${obstavoid_src_dirs} ${PCL_INCLUDE_DIRS}
                     )

foreach(obstavoid_lib_name ${obstavoid_components})
  add_subdirectory(${PROJECT_SOURCE_DIR}/${obstavoid_lib_name})
  set(obstavoid_libs ${obstavoid_libs} obstavoid_${obstavoid_lib_name})
endforeach()

set(EXE_NAME test)
add_executable(${EXE_NAME} obstavoid.cpp main.cpp)
#target_link_libraries(${EXE_NAME} opencv_core opencv_features2d opencv_highgui opencv_nonfree ${obstavoid_libs})
target_link_libraries(${EXE_NAME} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} gcrypt ${obstavoid_libs})
