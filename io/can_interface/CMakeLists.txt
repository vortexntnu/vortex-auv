add_library(can_interface STATIC
    src/can_interface.cpp
)

target_include_directories(can_interface
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(can_interface
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)
