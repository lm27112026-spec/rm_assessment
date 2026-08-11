add_library(communication STATIC
    frame.cpp
    mySerial.cpp
)

target_include_directories(communication PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_features(communication PUBLIC cxx_std_17)

if(WIN32)
    # Win32 API (CreateFileA, WriteFile, ReadFile) — no extra link needed
elseif(UNIX)
    target_link_libraries(communication PRIVATE pthread)
endif()