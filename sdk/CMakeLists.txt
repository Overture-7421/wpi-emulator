# Public, header-only interfaces shared by core, bridges, and year plugins.
# Must stay free of Qt, Box3D, and WPILib dependencies.
add_library(wpiemu_sdk INTERFACE)
add_library(wpiemu::sdk ALIAS wpiemu_sdk)
target_include_directories(wpiemu_sdk INTERFACE include)
target_compile_features(wpiemu_sdk INTERFACE cxx_std_20)
