cmake_minimum_required(VERSION 3.16)

project(rm_assessment_mycamera LANGUAGES CXX)

enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
  add_compile_options(/utf-8)
  set(CMAKE_MAP_IMPORTED_CONFIG_DEBUG Release)
endif()

if(WIN32 AND NOT OpenCV_DIR)
  set(OpenCV_DIR "D:/OpenCV/opencv/build" CACHE PATH "OpenCV package directory")
endif()

find_package(OpenCV QUIET)

if(WIN32)
  set(_YOLO_WITH_OPENVINO_DEFAULT ON)
else()
  set(_YOLO_WITH_OPENVINO_DEFAULT OFF)
endif()
option(YOLO_WITH_OPENVINO "Enable YOLOv5 + OpenVINO support" ${_YOLO_WITH_OPENVINO_DEFAULT})

if(MSVC AND YOLO_WITH_OPENVINO)
  set(CMAKE_CONFIGURATION_TYPES "Debug;Release;MinSizeRel;RelWithDebInfo" CACHE STRING "Available build configurations" FORCE)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
  add_compile_definitions($<$<CONFIG:Debug>:_ITERATOR_DEBUG_LEVEL=0>)
endif()

include(cmake/helpers.cmake)

add_subdirectory(src/MyCamera MyCamera)

if(YOLO_WITH_OPENVINO AND NOT OpenCV_FOUND)
  message(FATAL_ERROR "YOLO_WITH_OPENVINO requires OpenCV. Set OpenCV_DIR or disable YOLO_WITH_OPENVINO.")
endif()

if(OpenCV_FOUND)
  add_subdirectory(src/MyArmorTraditional MyArmorTraditional)
  add_subdirectory(communication)

  if(YOLO_WITH_OPENVINO)
    if(WIN32 AND NOT MSVC)
      message(FATAL_ERROR "YOLO_WITH_OPENVINO requires the MSVC toolchain because the configured OpenVINO package uses MSVC ABI. Recreate build with: cmake -S . -B build -G \"Visual Studio 17 2022\"")
    endif()
    if(WIN32 AND NOT OpenVINO_DIR)
      set(OpenVINO_DIR "D:/python/Lib/site-packages/openvino/cmake" CACHE PATH "OpenVINO package directory")
    endif()
    find_package(OpenVINO REQUIRED)

    set(_YOLO_RUNTIME_PATHS "")
    if(WIN32 AND DEFINED OpenCV_DIR)
      get_filename_component(OpenCV_BIN_DIR "${OpenCV_DIR}/x64/vc16/bin" ABSOLUTE)
      list(APPEND _YOLO_RUNTIME_PATHS "${OpenCV_BIN_DIR}")
    endif()
    if(WIN32 AND DEFINED OpenVINO_DIR)
      get_filename_component(OpenVINO_BIN_DIR "${OpenVINO_DIR}/../libs" ABSOLUTE)
      list(APPEND _YOLO_RUNTIME_PATHS "${OpenVINO_BIN_DIR}")
    endif()

    add_subdirectory(src/MyArmorYolo MyArmorYolo)
  endif()

  add_subdirectory(tasks)
else()
  message(STATUS "OpenCV not found: skipping armor_vision, YOLO, demos, and OpenCV-backed tests.")
endif()

add_subdirectory(tests)
