# Sources of bridges/vision/src/binding (work stream nt-binding): the run-time binding of the robot's ntcore
# C API behind INtApi (makeNtCoreApi), its staging in CMake and the real-ntcore tests.
target_sources(wpiemu_bridges_vision PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/nt_core_api.cpp)
