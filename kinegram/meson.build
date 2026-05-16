kinegram_inc = include_directories('include')

executable('kinegram',
  sources : files(
    'src/main.cpp',
    'src/config.cpp',
  ),
  include_directories : kinegram_inc,
  dependencies : [kinecore_dep, tomlplusplus_dep, spdlog_dep],
  install : true,
)
