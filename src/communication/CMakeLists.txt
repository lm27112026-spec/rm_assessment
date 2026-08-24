add_library(communication STATIC
    frame.cpp
)

target_include_directories(communication PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_features(communication PUBLIC cxx_std_17)

add_executable(frame_test tests/frame_test.cpp)
target_link_libraries(frame_test PRIVATE communication)
add_test(NAME communication_frame_test COMMAND frame_test)

add_executable(loopback_test tests/loopback_test.cpp)
target_link_libraries(loopback_test PRIVATE communication myserial)
add_test(NAME communication_loopback_test COMMAND loopback_test)
