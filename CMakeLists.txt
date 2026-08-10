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
else()
  message(STATUS "OpenCV not found: skipping armor_vision library and armor_demo executable.")
endif()

enable_testing()
add_test(NAME mycamera_test COMMAND mycamera_test)
set_tests_properties(mycamera_test PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
