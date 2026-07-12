find_package(Eigen3 REQUIRED)

add_library(eskf
  src/eskf.cpp
)

add_library(vortex::eskf ALIAS eskf)

target_include_directories(eskf
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(eskf
  PUBLIC
    Eigen3::Eigen
)

target_compile_features(eskf PUBLIC cxx_std_20)

target_compile_options(eskf
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)
