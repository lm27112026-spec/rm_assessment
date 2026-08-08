cmake_minimum_required(VERSION 3.16)

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(auto_aim OBJECT 
    armor.cpp
    classifier.cpp 
    detector.cpp
    solver.cpp
    target.cpp
    tracker.cpp
    yolo.cpp
    yolos/yolov5.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/mt_detector.cpp
)

target_link_libraries(auto_aim openvino::runtime)
