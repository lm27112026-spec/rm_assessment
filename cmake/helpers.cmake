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

# Copy OpenCV DLLs (and, for MinGW, the GCC runtime DLLs) next to an
# executable so it can be run directly without configuring PATH.
function(copy_opencv_runtime_dlls target_name)
  if(NOT WIN32)
    return()
  endif()

  # Locate the OpenCV DLL directory. OpenCV_DIR points at either the MSVC
  # build dir (DLLs under x64/vc16/bin) or a MinGW install (DLLs under ../bin).
  set(_opencv_bin_candidates "")
  if(DEFINED OpenCV_DIR)
    get_filename_component(_msvc_bin "${OpenCV_DIR}/x64/vc16/bin" ABSOLUTE)
    get_filename_component(_mingw_bin "${OpenCV_DIR}/../bin" ABSOLUTE)
    list(APPEND _opencv_bin_candidates "${_msvc_bin}" "${_mingw_bin}")
  endif()

  foreach(_bin IN LISTS _opencv_bin_candidates)
    if(EXISTS "${_bin}")
      file(GLOB _opencv_dlls CONFIGURE_DEPENDS "${_bin}/*.dll")
      if(_opencv_dlls)
        add_custom_command(TARGET ${target_name} POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_opencv_dlls} $<TARGET_FILE_DIR:${target_name}>
          COMMAND_EXPAND_LISTS)
      endif()
      break()
    endif()
  endforeach()

  # MinGW/GCC runtime DLLs (libgcc_s_seh-1.dll, libstdc++-6.dll, libwinpthread-1.dll).
  if(MINGW)
    get_filename_component(_gcc_bin "${CMAKE_CXX_COMPILER}" DIRECTORY)
    foreach(_dll_name IN ITEMS libgcc_s_seh-1.dll libgcc_s_dw2-1.dll libstdc++-6.dll libwinpthread-1.dll)
      set(_dll "${_gcc_bin}/${_dll_name}")
      if(EXISTS "${_dll}")
        add_custom_command(TARGET ${target_name} POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_dll}" $<TARGET_FILE_DIR:${target_name}>)
      endif()
    endforeach()
  endif()
endfunction()

# Copy MinGW/GCC runtime DLLs next to an executable. Used for targets that
# don't depend on OpenCV but still need the C++ runtime when built with MinGW.
function(copy_mingw_runtime_dlls target_name)
  if(NOT MINGW)
    return()
  endif()
  get_filename_component(_gcc_bin "${CMAKE_CXX_COMPILER}" DIRECTORY)
  foreach(_dll_name IN ITEMS libgcc_s_seh-1.dll libgcc_s_dw2-1.dll libstdc++-6.dll libwinpthread-1.dll)
    set(_dll "${_gcc_bin}/${_dll_name}")
    if(EXISTS "${_dll}")
      add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_dll}" $<TARGET_FILE_DIR:${target_name}>)
    endif()
  endforeach()
endfunction()

# Sync a YOLO demo executable (plus its runtime DLLs) into the MinGW build's
# tasks/ output directory. The MinGW build cannot compile OpenVINO-based
# targets, so the MSVC build mirrors its self-contained exe + DLLs into
# <source>/build/tasks/ so it can be run directly from there too.
function(sync_yolo_demo_to_build_tasks target_name)
  set(_dest "${CMAKE_SOURCE_DIR}/build/tasks")

  add_custom_command(TARGET ${target_name} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_dest}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:${target_name}>" "${_dest}"
    COMMENT "Syncing ${target_name} to ${_dest}")

  if(WIN32)
    foreach(runtime_dir IN LISTS _YOLO_RUNTIME_PATHS)
      if(EXISTS "${runtime_dir}")
        file(GLOB runtime_dlls CONFIGURE_DEPENDS "${runtime_dir}/*.dll")
        if(runtime_dlls)
          add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${runtime_dlls} "${_dest}"
            COMMAND_EXPAND_LISTS)
        endif()
      endif()
    endforeach()
  endif()
endfunction()

function(set_test_working_directory test_name)
  set_tests_properties(${test_name} PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endfunction()

function(set_opencv_test_path test_name)
  if(WIN32 AND DEFINED OpenCV_DIR)
    get_filename_component(OpenCV_BIN_DIR "${OpenCV_DIR}/x64/vc16/bin" ABSOLUTE)
    set_tests_properties(${test_name} PROPERTIES ENVIRONMENT "PATH=${OpenCV_BIN_DIR};$ENV{PATH}")
  endif()
endfunction()

function(set_yolo_test_path test_name)
  if(WIN32 AND _YOLO_RUNTIME_PATHS)
    list(JOIN _YOLO_RUNTIME_PATHS ";" _YOLO_RUNTIME_PATHS_JOINED)
    set_tests_properties(${test_name} PROPERTIES ENVIRONMENT "PATH=${_YOLO_RUNTIME_PATHS_JOINED};$ENV{PATH}")
  endif()
endfunction()
