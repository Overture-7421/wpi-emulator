# Sources of bridges/vision/src/photon (work stream photon-publisher): PhotonVision profile, generic codec, frame
# mapper and publisher. Public headers: include/wpiemu/bridges/vision/photon_*.h.
target_sources(wpiemu_bridges_vision PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/photon_codec.cpp
    ${CMAKE_CURRENT_LIST_DIR}/photon_mapper.cpp
    ${CMAKE_CURRENT_LIST_DIR}/photon_md5.cpp
    ${CMAKE_CURRENT_LIST_DIR}/photon_profile.cpp
    ${CMAKE_CURRENT_LIST_DIR}/photon_profile_data.cpp
    ${CMAKE_CURRENT_LIST_DIR}/photon_publisher.cpp
    ${CMAKE_CURRENT_LIST_DIR}/photon_vendor.cpp)
