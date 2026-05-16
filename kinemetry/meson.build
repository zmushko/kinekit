kinemetry_inc = include_directories('include')

executable('kinemetry',
  sources : files(
    'src/main.cpp',
    'src/config.cpp',
  ),
  include_directories : kinemetry_inc,
  dependencies : [kinecore_dep, tomlplusplus_dep, spdlog_dep],
  install : true,
)
