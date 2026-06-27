add_library(nucleus_interface
  src/nucleus_interface.cpp
)

add_library(vortex::nucleus_interface ALIAS nucleus_interface)

target_include_directories(nucleus_interface
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(nucleus_interface
  PUBLIC
    vortex_utils
  PRIVATE
    nortek_nucleus_driver::nortek_nucleus_driver
)

target_compile_features(nucleus_interface PUBLIC cxx_std_20)

target_compile_options(nucleus_interface
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)
