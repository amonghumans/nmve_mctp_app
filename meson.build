project(
    'nvme_mctp_test',
    'cpp',
    version: '1.0',
    default_options: [
        'cpp_std=c++17',
        'werror=true',
    ],
)
src = [
        'main.cpp',
]

#pkgconfigs deps
deps = [
    dependency('bull_logger'),
]

#find library that don't have pkgconfigs file
cpp = meson.get_compiler('cpp')
deps += [cpp.find_library('i2c')]

executable(
    'nvme_mctp_test',
    src,
    dependencies: deps,
    install: true
)
