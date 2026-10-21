# Robot editor (UI stream "ui-editor"): EditorPage + IEditorBackend + a fake backend for tests/demos.
# Static library linking only the SDK, the shared app/ui scene support (SDK + Qt) and Qt; the shell supplies the real IEditorBackend adapter.
# Layout: model/ (document, undo, pure edit ops), viewmodel/ (Qt item models, viewport controller),
# views/ (widgets), qml/ (Qt Quick 3D viewport). Models/view-models never include views.
# Build only this module with app/editor/standalone/CMakeLists.txt (see the comment in that file).

# The SDK header game/ui_descriptors.h matches AUTOUIC's ui_*.h pattern; this module has no .ui files.
set(CMAKE_AUTOUIC OFF)

set(WPIEMU_EDITOR_SOURCES
    editor_backend.cpp
    editor_page.cpp
    fake_editor_backend.cpp
    model/config_ops.cpp
    model/editor_document.cpp
    model/motion_links.cpp
    model/property_groups.cpp
    model/scene_builder.cpp
    model/slot_status.cpp
    model/spec_values.cpp
    model/transform_access.cpp
    viewmodel/detected_devices_model.cpp
    viewmodel/editor_controller.cpp
    viewmodel/issue_model.cpp
    viewmodel/project_tree_model.cpp
    viewmodel/scene_model.cpp
    viewmodel/selection_state.cpp
    viewmodel/viewport_controller.cpp
    views/activation_card.cpp
    views/bottom_panel.cpp
    views/device_panel.cpp
    views/device_row.cpp
    views/drive_card.cpp
    views/field_editors.cpp
    views/icon_factory.cpp
    views/inspector_panel.cpp
    views/project_tree_view.cpp
    views/property_form.cpp
    views/section_card.cpp
    views/toast_overlay.cpp
    views/viewport_widget.cpp
)

qt_add_library(wpiemu_app_editor STATIC ${WPIEMU_EDITOR_SOURCES})
qt_add_resources(wpiemu_app_editor wpiemu_editor_qml
    PREFIX "/wpiemu/editor"
    FILES qml/Viewport3D.qml)
target_include_directories(wpiemu_app_editor PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(wpiemu_app_editor
    PUBLIC wpiemu_sdk wpiemu_app_ui Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Quick Qt6::QuickWidgets Qt6::Quick3D)
wpiemu_target_defaults(wpiemu_app_editor)
target_compile_definitions(wpiemu_app_editor PRIVATE QT_NO_KEYWORDS)

get_filename_component(WPIEMU_EDITOR_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# ---- standalone demo: EditorPage + FakeEditorBackend -----------------------------------------------
if(WPIEMU_BUILD_UI_DEMOS)
    qt_add_executable(wpiemu_editor_demo demo/editor_demo.cpp)
    target_link_libraries(wpiemu_editor_demo PRIVATE wpiemu_app_editor)
    wpiemu_target_defaults(wpiemu_editor_demo)
endif()

# ---- tests (doctest; GUI smoke tests run with QT_QPA_PLATFORM=offscreen) ---------------------------
# Sources use the .cc extension on purpose: tests/CMakeLists.txt globs *.cpp into the Qt-free
# wpiemu_tests executable, which must not pick these up.
if(WPIEMU_BUILD_TESTS AND TARGET doctest::doctest)
    file(GLOB WPIEMU_EDITOR_TEST_SOURCES CONFIGURE_DEPENDS "${WPIEMU_EDITOR_REPO_ROOT}/tests/app_editor/*.cc")
    if(WPIEMU_EDITOR_TEST_SOURCES)
        add_executable(wpiemu_app_editor_tests ${WPIEMU_EDITOR_TEST_SOURCES})
        target_link_libraries(wpiemu_app_editor_tests PRIVATE wpiemu_app_editor doctest::doctest)
        target_compile_definitions(wpiemu_app_editor_tests PRIVATE DOCTEST_CONFIG_USE_STD_HEADERS)
        wpiemu_target_defaults(wpiemu_app_editor_tests)
        enable_testing()
        add_test(NAME app_editor.tests COMMAND wpiemu_app_editor_tests)
        set_tests_properties(app_editor.tests PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
        if(WIN32)
            get_target_property(_qt_core_loc Qt6::Core IMPORTED_LOCATION_RELEASE)
            if(NOT _qt_core_loc)
                get_target_property(_qt_core_loc Qt6::Core LOCATION)
            endif()
            get_filename_component(_qt_bin_dir "${_qt_core_loc}" DIRECTORY)
            set_property(TEST app_editor.tests APPEND PROPERTY ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${_qt_bin_dir}")
        endif()
    endif()
endif()
