# Respect an explicitly configured Qt5_DIR. Otherwise, look in the usual
# platform locations, including Homebrew's keg-only Qt5 formula on macOS.
if (NOT Qt5_DIR)
    set(_qt5_config_candidates)
    if (DEFINED ENV{QT_HOME} AND NOT "$ENV{QT_HOME}" STREQUAL "")
        list(APPEND _qt5_config_candidates
                "$ENV{QT_HOME}/lib/cmake/Qt5")
        if (WIN32)
            list(APPEND _qt5_config_candidates
                    "$ENV{QT_HOME}/5.15.2/msvc2019_64/lib/cmake/Qt5")
        elseif (APPLE)
            list(APPEND _qt5_config_candidates
                    "$ENV{QT_HOME}/5.15.2/clang_64/lib/cmake/Qt5")
        elseif (UNIX)
            list(APPEND _qt5_config_candidates
                    "$ENV{QT_HOME}/5.15.2/gcc_64/lib/cmake/Qt5")
        endif ()
    endif ()

    if (APPLE)
        if (DEFINED ENV{HOMEBREW_PREFIX} AND NOT "$ENV{HOMEBREW_PREFIX}" STREQUAL "")
            foreach (_qt_formula qt5 qt@5)
                list(APPEND _qt5_config_candidates
                        "$ENV{HOMEBREW_PREFIX}/opt/${_qt_formula}/lib/cmake/Qt5")
            endforeach ()
        endif ()

        find_program(_homebrew_executable NAMES brew HINTS /opt/homebrew/bin /usr/local/bin)
        if (_homebrew_executable)
            foreach (_qt_formula qt5 qt@5)
                execute_process(
                        COMMAND "${_homebrew_executable}" --prefix "${_qt_formula}"
                        RESULT_VARIABLE _qt_formula_result
                        OUTPUT_VARIABLE _qt_formula_prefix
                        OUTPUT_STRIP_TRAILING_WHITESPACE
                        ERROR_QUIET)
                if (_qt_formula_result EQUAL 0 AND _qt_formula_prefix)
                    list(APPEND _qt5_config_candidates
                            "${_qt_formula_prefix}/lib/cmake/Qt5")
                endif ()
            endforeach ()
        endif ()

        # Covers standard Apple Silicon and Intel Homebrew prefixes when CMake
        # is run in an environment where brew is not on PATH.
        list(APPEND _qt5_config_candidates
                /opt/homebrew/opt/qt5/lib/cmake/Qt5
                /opt/homebrew/opt/qt@5/lib/cmake/Qt5
                /usr/local/opt/qt5/lib/cmake/Qt5
                /usr/local/opt/qt@5/lib/cmake/Qt5)
    elseif (WIN32)
        list(APPEND _qt5_config_candidates
                "C:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5")
    endif ()

    foreach (_qt5_candidate IN LISTS _qt5_config_candidates)
        if (EXISTS "${_qt5_candidate}/Qt5Config.cmake")
            set(Qt5_DIR "${_qt5_candidate}" CACHE PATH "Qt5 CMake package directory" FORCE)
            message(STATUS "Using Qt5 CMake package: ${Qt5_DIR}")
            break ()
        endif ()
    endforeach ()
    unset(_qt5_config_candidates)
    unset(_qt5_candidate)
    unset(_homebrew_executable CACHE)
endif ()

set(MODULE_NAME Qt_module)

set(CMAKE_INCLUDE_CURRENT_DIR ON) #将编译目录包含到Include目录，否则编译器会找不到qt编译的这些头文件
set(CMAKE_AUTOMOC ON) # for meta object compiler
set(CMAKE_AUTORCC ON) # resource files
set(CMAKE_AUTOUIC ON) # UI files
set(CMAKE_AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/Resources/UI")

# 查找 Qt
find_package(Qt5 CONFIG REQUIRED COMPONENTS Core Gui Widgets OpenGL Charts Svg)
Qt5_add_resources(QRC_FILES "Resources/iGameQtMainWindow.qrc")

file(GLOB_RECURSE UI_FILES Resources/UI/*.ui)
Qt5_wrap_ui(UI_HEAD_FILES ${UI_FILES})

get_target_property(_qmake_executable Qt5::qmake IMPORTED_LOCATION)
get_filename_component(_qt_bin_dir "${_qmake_executable}" DIRECTORY)
find_program(DEPLOYQT_EXECUTABLE NAMES windeployqt macdeployqt HINTS "${_qt_bin_dir}")


file(GLOB_RECURSE QT_SRCS CONFIGURE_DEPENDS src/*.cpp include/*.h Plugin/*)

if (IGAME_BUILD_SHARED)
    add_library(${MODULE_NAME} SHARED
            ${QRC_FILES}
            ${UI_FILES}
            ${UI_HEAD_FILES}
            ${QT_SRCS}
    )
else ()
    add_library(${MODULE_NAME} STATIC
            ${QRC_FILES}
            ${UI_FILES}
            ${UI_HEAD_FILES}
            ${QT_SRCS}
    )
endif ()


set_source_files_properties(${QRC_FILES} PROPERTIES GENERATED ON)

target_include_directories(${MODULE_NAME} PUBLIC
        include
        Plugin
        "${CMAKE_BINARY_DIR}/Qt"
)
set(LINK_LIBS)

list(APPEND LINK_LIBS
        Qt5::OpenGL
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::Charts
        Qt5::Svg
        iGameCore
)

if (EMSCRIPTEN)
    set(LINK_OPTS)
    list(APPEND LINK_LIBS Qt5::Platform)
    list(APPEND LINK_OPTS "SHELL:-s FULL_ES2=1")
    target_link_options(${MODULE_NAME} PUBLIC ${LINK_OPTS})
endif ()

target_link_libraries(${MODULE_NAME} PUBLIC ${LINK_LIBS})
if (WIN32)
    #  内部库在cmake构建时定义IG_QT_COMPILE_EXPORT，使得模式切换为dllexport, 否则外部库在使用编译这个动态库代码时将其模式切换为dllimport
    target_compile_definitions(${MODULE_NAME} PRIVATE IG_QT_COMPILE_EXPORT)
    #  由于Windows不会搜索其他目录下的dll文件因此要使用cmake指令将dll移至exe目录
    #  将QT模块编译成的可执行文件(.exe)、静态库(.lib)、动态库(.dll)的输出移至放入主程序源码目录
    set_property(TARGET ${MODULE_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
    set_property(TARGET ${MODULE_NAME} PROPERTY ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
    set_property(TARGET ${MODULE_NAME} PROPERTY LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
    set_property(TARGET ${MODULE_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR})
    set_property(TARGET ${MODULE_NAME} PROPERTY ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR})
    set_property(TARGET ${MODULE_NAME} PROPERTY LIBRARY_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR})
    set_property(TARGET ${MODULE_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR})
    set_property(TARGET ${MODULE_NAME} PROPERTY ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR})
    set_property(TARGET ${MODULE_NAME} PROPERTY LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR})
    get_target_property(Qt5Core_LOCATION Qt5::Core LOCATION)
    get_filename_component(Qt5_INSTALL_DIR ${Qt5Core_LOCATION} DIRECTORY)
endif ()
