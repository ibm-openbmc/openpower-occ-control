gtest_dep = dependency('gtest', main: true, disabler: true, required: false)
gmock_dep = dependency('gmock', disabler: true, required: false)
if not gtest_dep.found() or not gmock_dep.found()
    gtest_proj = import('cmake').subproject('googletest', required: false)
    if gtest_proj.found()
        gtest_dep = declare_dependency(
            dependencies: [
                dependency('threads'),
                gtest_proj.dependency('gtest'),
                gtest_proj.dependency('gtest_main'),
            ],
        )
        gmock_dep = gtest_proj.dependency('gmock')
    else
        assert(
            not get_option('tests').allowed(),
            'Googletest is required if tests are enabled',
        )
    endif
endif

test_sources = [
    '../occ_pass_through.cpp',
    '../occ_manager.cpp',
    '../occ_object.cpp',
    '../occ_device.cpp',
    '../occ_errors.cpp',
    '../occ_ffdc.cpp',
    '../occ_presence.cpp',
    '../occ_command.cpp',
    '../occ_dbus.cpp',
    '../pldm.cpp',
    '../powercap.cpp',
    '../powermode.cpp',
    '../utils.cpp',
    occ_sensor_hpp,
    '../occ_poll_handler.cpp',
]

if get_option('app-poll-support').allowed()
    test_sources += ['../occ_poll_app_handler.cpp']
else
    test_sources += ['../occ_poll_kernel_handler.cpp']
endif


tests = [
    'error_files_tests.cpp',
    'occ_dbus_test.cpp',
    'utest.cpp',
    'error_readfile_race_test.cpp',
]

pthread_dep = dependency('threads')

foreach t : tests
    test(
        t,
        executable(
            t.underscorify(),
            t,
            test_sources,
            include_directories: '..',
            dependencies: [gtest_dep, gmock_dep, pthread_dep, deps],
        ),
        workdir: meson.current_source_dir(),
    )
endforeach

# occ_sensor_path_test only exercises BuildTempDbusPaths — it does not need
# the full occ_manager / powercap / pldm stack, so it gets its own minimal
# source list to avoid unrelated compile errors in those files.
sensor_path_test_sources = [
    '../occ_dbus.cpp',
    '../utils.cpp',
    '../occ_command.cpp',
    '../occ_errors.cpp',
    '../occ_ffdc.cpp',
    '../occ_poll_handler.cpp',
    occ_sensor_hpp,
]
if get_option('app-poll-support').allowed()
    sensor_path_test_sources += ['../occ_poll_app_handler.cpp']
else
    sensor_path_test_sources += [
        '../occ_poll_kernel_handler.cpp',
        'occ_sensor_path_test_stubs.cpp',
    ]
endif

test(
    'occ_sensor_path_test.cpp',
    executable(
        'occ_sensor_path_test_cpp',
        'occ_sensor_path_test.cpp',
        sensor_path_test_sources,
        include_directories: '..',
        dependencies: [gtest_dep, gmock_dep, pthread_dep, deps],
    ),
    workdir: meson.current_source_dir(),
    args: ['--gtest_print_time=0'],
    verbose: true,
)

# When the build uses -Db_sanitize=address,undefined the ASAN runtime aborts
# on startup under Valgrind because it is not first in the dynamic-linker
# preload chain.  Defining a 'valgrind' test setup here causes unit-test.py
# to use --setup instead of --wrapper, letting us pass the suppression env
# var that prevents ASAN from treating the ordering as fatal.
add_test_setup(
    'valgrind',
    exe_wrapper: ['valgrind', '--error-exitcode=1'],
    env: {
        'ASAN_OPTIONS': 'verify_asan_link_order=0:halt_on_error=1:abort_on_error=1:print_summary=1',
        'UBSAN_OPTIONS': 'halt_on_error=1:abort_on_error=1:print_summary=1:print_stacktrace=1',
    },
    timeout_multiplier: 10,
)
