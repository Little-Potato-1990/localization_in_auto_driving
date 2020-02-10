find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
include_directories(${YAML_CPP_INCLUDEDIR})
list(APPEND ALL_TARGET_LIBRARIES ${YAML_CPP_LIBRARIES})