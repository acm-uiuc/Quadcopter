set(LIB_NAME ${obstavoid_lib_name})
set(LINK_LIBS ${OpenCV_LIBRARIES})

set(LIB_SRC ContourKeypoints.cpp
            )


add_library(obstavoid_${LIB_NAME} ${LIB_SRC})
target_link_libraries(obstavoid_${LIB_NAME} ${LINK_LIBS})

set(EXE_NAME get_contours)
add_executable(${EXE_NAME} main.cpp)
target_link_libraries(${EXE_NAME} ${LINK_LIBS} obstavoid_${LIB_NAME})


