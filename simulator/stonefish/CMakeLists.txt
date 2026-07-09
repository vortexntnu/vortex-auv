add_library(vortex_stonefish
  src/simulator.cpp
)

add_library(vortex::stonefish ALIAS vortex_stonefish)

target_compile_features(vortex_stonefish
  PUBLIC
    cxx_std_20
)

target_compile_definitions(vortex_stonefish
  PUBLIC
    BT_USE_DOUBLE_PRECISION
)

target_include_directories(vortex_stonefish
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(vortex_stonefish
  SYSTEM PUBLIC
    /usr/local/include
    /usr/local/include/Stonefish
)

find_library(STONEFISH_LIBRARY
  NAMES Stonefish
  PATHS /usr/local/lib
  REQUIRED
)

target_link_libraries(vortex_stonefish
  PUBLIC
    ${STONEFISH_LIBRARY}
)
