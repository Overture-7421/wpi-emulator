# Common settings for first-party code.
#   wpiemu_option_warnings_as_errors(<default>)
#   wpiemu_target_defaults(<target>)

# Declares WPIEMU_WARNINGS_AS_ERRORS, which wpiemu_target_defaults reads. Only the default differs per build: the full
# build (presets, CI) keeps it OFF, the standalone editor build is a developer tool and keeps it ON.
function(wpiemu_option_warnings_as_errors default)
    option(WPIEMU_WARNINGS_AS_ERRORS "Treat warnings in first-party code as errors" ${default})
endfunction()

function(wpiemu_target_defaults target)
    get_target_property(type ${target} TYPE)
    if(type STREQUAL "INTERFACE_LIBRARY")
        return()
    endif()

    target_compile_features(${target} PUBLIC cxx_std_20)

    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8 /Zc:__cplusplus)
        if(WPIEMU_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
        if(WPIEMU_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()
