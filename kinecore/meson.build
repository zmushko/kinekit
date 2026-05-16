kinecore_inc = include_directories('include')

kinecore_lib = library('kinecore',
  sources : files('src/version.cpp'),
  include_directories : kinecore_inc,
  install : true,
)

kinecore_dep = declare_dependency(
  link_with : kinecore_lib,
  include_directories : kinecore_inc,
)

install_headers('include/kinecore/version.hpp', subdir : 'kinecore')
