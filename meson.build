project('kinekit', 'cpp',
  version : '0.1.0',
  license : 'TBD',
  default_options : [
    'cpp_std=c++17',
    'warning_level=2',
    'buildtype=debugoptimized',
  ],
)

cpp = meson.get_compiler('cpp')

# Shared dependencies declared once so every subdir picks the same
# configuration.

# toml++ header-only: pull in the headers shipped by libtomlplusplus-dev
# but skip linking against libtomlplusplus.so so binaries stay
# self-contained on the Pi.
assert(cpp.has_header('toml++/toml.h'), 'toml++/toml.h not found — install libtomlplusplus-dev')
tomlplusplus_dep = declare_dependency(
  compile_args : ['-DTOML_HEADER_ONLY=1'],
)

# spdlog header-only: same idea — inline into each module's .so so the
# Pi doesn't need libspdlog installed.
assert(cpp.has_header('spdlog/spdlog.h'), 'spdlog/spdlog.h not found — install libspdlog-dev')
spdlog_dep = declare_dependency(
  compile_args : ['-DSPDLOG_HEADER_ONLY'],
)

subdir('kinecore')
subdir('kinegram')
subdir('kinemetry')
