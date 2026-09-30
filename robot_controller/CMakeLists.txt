cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME robot_controller)
set(PROJECT_VERSION 1.0.0)
project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION ${PROJECT_VERSION}
)

# ── jrl-cmakemodules v2 ───────────────────────────────────────────────────────
include(../cmake/get-jrl-cmakemodules.cmake)

jrl_configure_defaults()
set(CMAKE_CXX_STANDARD 17)

# ── Options ───────────────────────────────────────────────────────────────────
jrl_option(BUILD_TESTING "Build tests" OFF)
jrl_option(ROBOT_CONTROLLER_WITH_MC_RTC "Build with mc_rtc backend" ON)

# ── Build ─────────────────────────────────────────────────────────────────────
add_subdirectory(src)

if(BUILD_TESTING)
  jrl_include_ctest()
  add_subdirectory(tests)
endif()

# ── Config ────────────────────────────────────────────────────────────────────
include(CMakePackageConfigHelpers)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
)

install(
  EXPORT ${PROJECT_NAME}Targets FILE ${PROJECT_NAME}Targets.cmake NAMESPACE
                                ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# ── Summary ───────────────────────────────────────────────────────────────────
jrl_print_options_summary()
