project('sdl2_windows', 'cpp')

fs = import('fs')

new_dir = join_paths('C:/TDT4102-mingw64','SDL2','sdl2_lib')
old_dir = join_paths(meson.current_source_dir(), 'lib')
#Ensuring backwards compatability in transition when SDL2 can be both places
dir_to_use = fs.exists(new_dir) ? new_dir : old_dir

cxx = meson.get_compiler('cpp')
sdl2_windows_dep = declare_dependency(
  dependencies : [cxx.find_library('SDL2main', dirs : dir_to_use), 
                  cxx.find_library('SDL2', dirs : dir_to_use)],
  link_args: ['-lmingw32'],
  include_directories : include_directories(join_paths('include', 'SDL2')))

