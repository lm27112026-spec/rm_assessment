cmake_minimum_required(VERSION 3.16)

project(rm_assessment_mycamera LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV QUIET)

add_library(mycamera io/myCamera.cpp)
target_include_directories(mycamera PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(OpenCV_FOUND)
  target_include_directories(mycamera PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(mycamera PUBLIC ${OpenCV_LIBS})
else()
  target_compile_definitions(mycamera PUBLIC MY_CAMERA_FORCE_FAKE=1)
endif()

function(add_standalone_cpp target_name source_file)
  add_executable(${target_name} ${source_file})
  target_link_libraries(${target_name} PRIVATE mycamera)
endfunction()

add_standalone_cpp(example io/example.cpp)
add_standalone_cpp(mycamera_test tests/myCamera_test.cpp)

if(OpenCV_FOUND)
  add_library(armor_vision
    src/armor.cpp
    src/detector.cpp
    src/digit_recognizer.cpp
    src/tracker.cpp)
  target_include_directories(armor_vision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_include_directories(armor_vision PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(armor_vision PUBLIC ${OpenCV_LIBS})

  add_executable(armor_demo tasks/armor_demo.cpp)
  target_link_libraries(armor_demo PRIVATE armor_vision)

  add_executable(armor_vision_test tests/armor_vision_test.cpp)
  target_link_libraries(armor_vision_test PRIVATE armor_vision)
else()
  message(STATUS "OpenCV not found: skipping armor_vision library and armor_demo executable.")
endif()

enable_testing()
add_test(NAME mycamera_test COMMAND mycamera_test)
set_tests_properties(mycamera_test PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

if(OpenCV_FOUND)
  add_test(NAME armor_vision_test COMMAND armor_vision_test)
  set_tests_properties(armor_vision_test PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  if(WIN32 AND DEFINED OpenCV_DIR)
    get_filename_component(OpenCV_BIN_DIR "${OpenCV_DIR}/x64/vc16/bin" ABSOLUTE)
    set_tests_properties(armor_vision_test PROPERTIES
      ENVIRONMENT "PATH=${OpenCV_BIN_DIR};$ENV{PATH}")
  endif()
endif()

option(YOLO_WITH_OPENVINO "Enable YOLOv5 + OpenVINO support" OFF)

if(YOLO_WITH_OPENVINO)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "YOLO_WITH_OPENVINO requires OpenCV. Set OpenCV_DIR or disable YOLO_WITH_OPENVINO.")
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

  function(copy_yolo_runtime_dlls target_name)
    if(WIN32)
      foreach(runtime_dir IN LISTS _YOLO_RUNTIME_PATHS)
        if(EXISTS "${runtime_dir}")
          file(GLOB runtime_dlls CONFIGURE_DEPENDS "${runtime_dir}/*.dll")
          if(runtime_dlls)
            add_custom_command(TARGET ${target_name} POST_BUILD
              COMMAND ${CMAKE_COMMAND} -E copy_if_different ${runtime_dlls} $<TARGET_FILE_DIR:${target_name}>
              COMMAND_EXPAND_LISTS)
          endif()
        endif()
      endforeach()
    endif()
  endfunction()

  add_library(yolov5
    src/yolov5/yolov5_utils.cpp
    src/yolov5/yolov5.cpp)
  target_include_directories(yolov5 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_include_directories(yolov5 PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(yolov5 PUBLIC openvino::runtime ${OpenCV_LIBS})

  add_executable(yolov5_test tests/yolov5_test.cpp)
  target_link_libraries(yolov5_test PRIVATE yolov5)
  copy_yolo_runtime_dlls(yolov5_test)

  add_executable(yolov5_armor_demo tasks/yolov5_armor_demo.cpp)
  target_link_libraries(yolov5_armor_demo PRIVATE yolov5 armor_vision)
  copy_yolo_runtime_dlls(yolov5_armor_demo)

  add_test(NAME yolov5_test COMMAND yolov5_test)
  set_tests_properties(yolov5_test PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  if(WIN32)
    if(_YOLO_RUNTIME_PATHS)
      list(JOIN _YOLO_RUNTIME_PATHS ";" _YOLO_RUNTIME_PATHS_JOINED)
      set_tests_properties(yolov5_test PROPERTIES
        ENVIRONMENT "PATH=${_YOLO_RUNTIME_PATHS_JOINED};$ENV{PATH}")
    endif()
  endif()
endif()
