kinecore_inc = include_directories('include')

kinecore_lib = library('kinecore',
  sources : files(
    'src/version.cpp',
    'src/config.cpp',
  ),
  include_directories : kinecore_inc,
  dependencies : [tomlplusplus_dep, spdlog_dep],
  install : true,
)

# kinecore_dep does NOT propagate tomlplusplus — toml++ is an implementation
# detail of the library, hidden behind kinecore's public headers.
kinecore_dep = declare_dependency(
  link_with : kinecore_lib,
  include_directories : kinecore_inc,
)

install_headers(
  'include/kinecore/version.hpp',
  'include/kinecore/config.hpp',
  subdir : 'kinecore',
)
