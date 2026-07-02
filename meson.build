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

# spdlog header-only. Debian Trixie ships spdlog configured to use an
# external libfmt (-DSPDLOG_FMT_EXTERNAL is hard-coded in its headers),
# so the header-only path still pulls in fmt symbols that must be
# linked. We therefore bundle libfmt alongside the header-only spdlog —
# fmt itself is small and self-contained, so the combined Pi-side
# footprint stays tiny and we still don't need to install libspdlog3 on
# the device.
assert(cpp.has_header('spdlog/spdlog.h'), 'spdlog/spdlog.h not found — install libspdlog-dev')
fmt_dep = dependency('fmt', required : true)
spdlog_dep = declare_dependency(
  compile_args : ['-DSPDLOG_HEADER_ONLY'],
  dependencies : [fmt_dep],
)

subdir('kinecore')
subdir('kinegram')
subdir('kinemetry')
