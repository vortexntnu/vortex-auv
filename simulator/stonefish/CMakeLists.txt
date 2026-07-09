add_library(vortex_stonefish
  src/simulator.cpp
)

add_library(vortex::stonefish ALIAS vortex_stonefish)

target_compile_features(vortex_stonefish
  PUBLIC
    cxx_std_20
)

target_include_directories(vortex_stonefish
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /usr/local/include
    /usr/local/include/Stonefish
)

target_link_libraries(vortex_stonefish
  PUBLIC
    stonefish
)
