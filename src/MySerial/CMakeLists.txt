add_library(myserial STATIC
  MySerial.cpp
)

target_include_directories(myserial PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/src
)

if(UNIX)
  target_link_libraries(myserial PRIVATE pthread)
endif()
