cmake_minimum_required(VERSION 3.16)

project(rm_assessment_mycamera LANGUAGES CXX)

enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV QUIET)

option(YOLO_WITH_OPENVINO "Enable YOLOv5 + OpenVINO support" OFF)

include(cmake/helpers.cmake)

add_subdirectory(src/MyCamera MyCamera)

if(YOLO_WITH_OPENVINO AND NOT OpenCV_FOUND)
  message(FATAL_ERROR "YOLO_WITH_OPENVINO requires OpenCV. Set OpenCV_DIR or disable YOLO_WITH_OPENVINO.")
endif()

if(OpenCV_FOUND)
  add_subdirectory(src/MyArmorTraditional MyArmorTraditional)
  add_subdirectory(communication)

  if(YOLO_WITH_OPENVINO)
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
