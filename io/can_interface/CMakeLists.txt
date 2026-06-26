add_library(can_interface STATIC
    src/can_interface.cpp
)

add_library(vortex::can_interface ALIAS can_interface)

target_include_directories(can_interface
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_options(can_interface
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)
