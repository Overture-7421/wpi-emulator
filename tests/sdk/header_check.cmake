# Self-contained include check: every public SDK header gets its own translation unit that
# includes it (twice, to prove the include guard) and nothing else. The objects are also linked
# into wpiemu_tests, so non-inline definitions in headers surface as duplicate-symbol (ODR) errors.
file(GLOB_RECURSE WPIEMU_SDK_HEADERS
    RELATIVE "${CMAKE_SOURCE_DIR}/sdk/include"
    CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/sdk/include/*.h")

set(WPIEMU_SDK_HEADER_CHECK_SOURCES)
foreach(header IN LISTS WPIEMU_SDK_HEADERS)
    string(MAKE_C_IDENTIFIER "${header}" header_id)
    set(source "${CMAKE_CURRENT_BINARY_DIR}/sdk_header_check/${header_id}.cpp")
    file(GENERATE OUTPUT "${source}"
        CONTENT "#include \"${header}\"\n#include \"${header}\"\n")
    list(APPEND WPIEMU_SDK_HEADER_CHECK_SOURCES "${source}")
endforeach()

add_library(wpiemu_sdk_header_check OBJECT ${WPIEMU_SDK_HEADER_CHECK_SOURCES})
target_link_libraries(wpiemu_sdk_header_check PRIVATE wpiemu_sdk)
# qt_standard_project_setup() turns the Qt autogens on globally; these generated TUs never need them.
set_target_properties(wpiemu_sdk_header_check PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
wpiemu_target_defaults(wpiemu_sdk_header_check)
