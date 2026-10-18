# Sources of bridges/vision/src/limelight (work stream limelight-publisher): key table (include/.../limelight_keys.h), geometry,
# MegaTag, the per-frame result, the orientation history and the publisher.
target_sources(wpiemu_bridges_vision PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/limelight_math.cpp
    ${CMAKE_CURRENT_LIST_DIR}/limelight_orientation.cpp
    ${CMAKE_CURRENT_LIST_DIR}/limelight_vendor.cpp)
