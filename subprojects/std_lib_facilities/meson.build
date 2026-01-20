project('std_lib_facilities', ['c', 'cpp'], version: '0.01', default_options: ['cpp_std=c++20', 'c_std=c17', 'default_library=static', 'buildtype=debugoptimized'])

incdir = include_directories('.')
install_subdir('include', install_dir: '.')

std_lib_facilities_dep = declare_dependency(include_directories: incdir)

