libcamera_dep = dependency('libcamera', required : true)

kinecore_inc = include_directories('include')

kinecore_lib = library('kinecore',
  sources : files(
    'src/version.cpp',
    'src/config.cpp',
    'src/fs.cpp',
    'src/camera.cpp',
  ),
  include_directories : kinecore_inc,
  dependencies : [tomlplusplus_dep, spdlog_dep, libcamera_dep],
  install : true,
)

kinecore_dep = declare_dependency(
  link_with : kinecore_lib,
  include_directories : kinecore_inc,
)

install_headers(
  'include/kinecore/version.hpp',
  'include/kinecore/config.hpp',
  'include/kinecore/frame.hpp',
  'include/kinecore/camera.hpp',
  'include/kinecore/fs.hpp',
  'include/kinecore/v4l2.hpp',
  subdir : 'kinecore',
)
