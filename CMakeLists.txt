cmake_minimum_required(VERSION 3.4...3.24)
project(patrol_car)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "-g -Wall -mcpu=native -flto -pthread")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(INCLUDE_PATH "/usr/local/include")
set(LIB_PATH "/usr/local/lib")
SET(CMAKE_CXX_COMPILER "g++")
#---------------------------------------------------------------------
#       [ Include、Lib  : Define ] ==> []
#---------------------------------------------------------------------
set(PROJECT_SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)

set(COMMON_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/")

set(COMMON_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include"
                        "${PROJECT_SOURCE_DIR}/config"
                        "${PROJECT_SOURCE_DIR}/driver" 
                        "${PROJECT_SOURCE_DIR}/lib" 
                        "${PROJECT_SOURCE_DIR}/detection"
                        "${PROJECT_SOURCE_DIR}/element"
                        "${PROJECT_SOURCE_DIR}/tool"
                        "${PROJECT_SOURCE_DIR}/control"
                        "${PROJECT_SOURCE_DIR}/utils"
                        "${PROJECT_SOURCE_DIR}/TUI"
                                  )


link_directories(${COMMON_LIB_DIR})
include_directories( ${COMMON_INCLUDE_DIR} )
#---------------------------------------------------------------------
#       [ Include、Lib  : Path ] ==> [ glib opencv ]
#---------------------------------------------------------------------
find_package(PkgConfig REQUIRED)
pkg_search_module(GLIB REQUIRED IMPORTED_TARGET glib-2.0)

pkg_search_module( OpenCV REQUIRED IMPORTED_TARGET opencv )    

pkg_search_module(SERIAL REQUIRED IMPORTED_TARGET libserial)

# find ppnc
pkg_search_module(PPNC REQUIRED IMPORTED_TARGET ppnc)


# find onnx
pkg_search_module(ONNX REQUIRED IMPORTED_TARGET onnx)

#find ffmpeg
pkg_check_modules(ffmpeg REQUIRED IMPORTED_TARGET GLOBAL
    libavfilter
    libavformat
    libavdevice
    libavcodec
    libavutil
    libswresample
    libswscale
)




#---------------------------------------------------------------------
#               [ bin ] ==> [ tool ]
#---------------------------------------------------------------------
# 图像采集
set(COL_PROJECT_NAME "collection")
set(COL_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tool/collection.cpp)
add_executable(${COL_PROJECT_NAME} ${COL_PROJECT_SOURCES})
target_link_libraries(${COL_PROJECT_NAME} pthread )
target_link_libraries(${COL_PROJECT_NAME} PkgConfig::OpenCV)
target_link_libraries(${COL_PROJECT_NAME} PkgConfig::SERIAL)

# 图像合成
set(IMG2V_PROJECT_NAME "img2video")
set(IMG2V_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tool/img2video.cpp)
add_executable(${IMG2V_PROJECT_NAME} ${IMG2V_PROJECT_SOURCES})
target_link_libraries(${IMG2V_PROJECT_NAME} pthread )
target_link_libraries(${IMG2V_PROJECT_NAME} PkgConfig::OpenCV)

# 相机标定
set(CAL_PROJECT_NAME "calibration")
set(CAL_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tool/calibration.cpp)
add_executable(${CAL_PROJECT_NAME} ${CAL_PROJECT_SOURCES})
target_link_libraries(${CAL_PROJECT_NAME} pthread )
target_link_libraries(${CAL_PROJECT_NAME} PkgConfig::OpenCV)

# 相机测试
set(CAM_PROJECT_NAME "camera")
set(CAM_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tool/camera.cpp)
add_executable(${CAM_PROJECT_NAME} ${CAM_PROJECT_SOURCES})
target_link_libraries(${CAM_PROJECT_NAME} pthread )
target_link_libraries(${CAM_PROJECT_NAME} PkgConfig::OpenCV)

#---------------------------------------------------------------------
#               [ bin ] ==> [ main ]
#---------------------------------------------------------------------


aux_source_directory(${PROJECT_SOURCE_DIR}/element ELEMENT)
aux_source_directory(${PROJECT_SOURCE_DIR}/detection DETECTION)
aux_source_directory(${PROJECT_SOURCE_DIR}/utils UTILS)
aux_source_directory(${PROJECT_SOURCE_DIR}/control CONTROL)
aux_source_directory(${PROJECT_SOURCE_DIR}/config CONFIG)
aux_source_directory(${PROJECT_SOURCE_DIR}/include INC)
aux_source_directory(${PROJECT_SOURCE_DIR}/vision VISION)
aux_source_directory(${PROJECT_SOURCE_DIR}/driver DRIVER)
set(SMARTCAR_SRC ${PROJECT_SOURCE_DIR}/SmartCar.cpp )

add_library(SMARTCAR STATIC ${SMARTCAR_SRC} ${ELEMENT} ${DETECTION} ${UTILS} 
${CONTROL} ${CONFIG} ${INC} ${VISION} ${DRIVER})
target_link_libraries(SMARTCAR PUBLIC PkgConfig::PPNC)
target_link_libraries(SMARTCAR PUBLIC PkgConfig::ONNX)
target_link_libraries(SMARTCAR PUBLIC PkgConfig::OpenCV)
target_link_libraries(SMARTCAR PUBLIC pthread )
target_link_libraries(SMARTCAR PUBLIC PkgConfig::SERIAL)
target_link_libraries(SMARTCAR PUBLIC PkgConfig::ffmpeg)
target_link_libraries(SMARTCAR PUBLIC PkgConfig::GLIB)

set(ICAR_PROJECT_NAME "icar")
set(ICAR_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/icar.cpp)
add_executable(${ICAR_PROJECT_NAME} ${ICAR_PROJECT_SOURCES})
target_link_libraries(${ICAR_PROJECT_NAME} PUBLIC SMARTCAR)
target_link_libraries(${ICAR_PROJECT_NAME} PUBLIC PkgConfig::OpenCV)


#-------------
#Test
#-------------

#add_subdirectory(test)


set(TEST_PROJECT_NAME "test")
set(TEST_PROJECT_SOURCES test/main.cpp)
add_executable(${TEST_PROJECT_NAME} ${TEST_PROJECT_SOURCES})
target_link_libraries(${TEST_PROJECT_NAME} PUBLIC SMARTCAR)

#八邻域测试
set(TEST_PROJECT_NAME "test2")
set(TEST_PROJECT_SOURCES test/icar_with8.cpp)
add_executable(${TEST_PROJECT_NAME} ${TEST_PROJECT_SOURCES})
target_link_libraries(${TEST_PROJECT_NAME} PUBLIC SMARTCAR)

#旧代码测试
set(TEST_PROJECT_NAME "test3")
set(TEST_PROJECT_SOURCES test/old_code_test.cpp)
add_executable(${TEST_PROJECT_NAME} ${TEST_PROJECT_SOURCES})
target_link_libraries(${TEST_PROJECT_NAME} PUBLIC SMARTCAR)
