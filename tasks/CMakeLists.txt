add_executable(armor_demo armor_demo.cpp)
target_link_libraries(armor_demo PRIVATE armor_vision)
copy_opencv_runtime_dlls(armor_demo)

add_executable(serial_demo serial_demo.cpp)
target_link_libraries(serial_demo PRIVATE armor_vision mycamera communication)
copy_opencv_runtime_dlls(serial_demo)
if(MSVC)
  # Existing Chinese comments in headers may trigger MSVC C4819 codepage warnings.
  target_compile_options(serial_demo PRIVATE /wd4819)
endif()

add_executable(serial_loopback serial_loopback.cpp)
target_link_libraries(serial_loopback PRIVATE communication)
copy_mingw_runtime_dlls(serial_loopback)

if(TARGET yolov5)
  add_executable(yolov5_armor_demo yolov5_armor_demo.cpp)
  target_link_libraries(yolov5_armor_demo PRIVATE yolov5)
  copy_yolo_runtime_dlls(yolov5_armor_demo)
  sync_yolo_demo_to_build_tasks(yolov5_armor_demo)

  add_executable(yolov5_pnp_demo yolov5_pnp_demo.cpp)
  target_link_libraries(yolov5_pnp_demo PRIVATE yolov5)
  copy_yolo_runtime_dlls(yolov5_pnp_demo)
  sync_yolo_demo_to_build_tasks(yolov5_pnp_demo)
endif()
