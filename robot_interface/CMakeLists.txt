cmake_minimum_required(VERSION 3.22)

# Setting PROJECT_NAME of robot_interface as unified_robot_interface for easy identification when using find_package() in downstream projects
set(PROJECT_NAME unified_robot_interface)
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

# ── Dependencies ──────────────────────────────────────────────────────────────
jrl_find_package(mc_rtc REQUIRED)

# ── Build ─────────────────────────────────────────────────────────────────────
add_subdirectory(src)

if(BUILD_TESTING)
  jrl_include_ctest()
  add_subdirectory(tests)
endif()

# ── Install ───────────────────────────────────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)
install(FILES cmake/AddRobotDriver.cmake
        DESTINATION lib/cmake/unified_robot_interface
)

jrl_add_export_component(
  NAME unified_robot_interface TARGETS robot_driver_api robot_driver_loader
)

jrl_export_package(
  PACKAGE_CONFIG_EXTRA_CONTENT
  "include(\"\${CMAKE_CURRENT_LIST_DIR}/AddRobotDriver.cmake\")"
)

jrl_print_dependencies_summary()
jrl_print_options_summary()
