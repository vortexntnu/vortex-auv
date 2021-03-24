
find_package(Eigen3 REQUIRED)
add_library(eskf_lib
        common.cpp
        ESKF.cpp)

target_include_directories(eskf_lib
        PUBLIC
        ../include/eskf
	${EIGEN3_INCLUDE_DIR}
        PRIVATE
        ../src
        )


#target_compile_features(ESKF_lib PUBLIC cxx_std_11)
