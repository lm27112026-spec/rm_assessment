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

enable_testing()
add_test(NAME mycamera_test COMMAND mycamera_test)
set_tests_properties(mycamera_test PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
