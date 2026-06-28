add_library(joystick_interface
  src/joystick_interface.cpp
)

add_library(vortex::joystick_interface ALIAS joystick_interface)

target_include_directories(joystick_interface
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(joystick_interface
  PUBLIC
    vortex_utils
)

target_compile_features(joystick_interface PUBLIC cxx_std_20)

target_compile_definitions(joystick_interface
  PUBLIC
    BOOST_ERROR_CODE_HEADER_ONLY
)

target_compile_options(joystick_interface
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)
