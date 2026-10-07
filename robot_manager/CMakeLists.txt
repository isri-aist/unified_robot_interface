cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME robot_manager)
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
jrl_option(
  USE_SETCAP "Use setcap to set real-time permissions on the binary" ON
)

# ── Dependencies ──────────────────────────────────────────────────────────────
jrl_find_package(mc_rtc REQUIRED)
jrl_find_package(Threads REQUIRED)
jrl_find_package(fmt REQUIRED)

if(USE_SETCAP)
  find_program(SETCAP setcap)
  find_program(SUDO sudo)
  if(NOT SUDO)
    message(FATAL_ERROR "sudo must be available on the system")
  else()
    message(STATUS "sudo command: ${SUDO}")
  endif()
endif()

# ── Build ─────────────────────────────────────────────────────────────────────
add_subdirectory(src)

if(BUILD_TESTING)
  jrl_include_ctest()
  add_subdirectory(tests)
endif()

install(DIRECTORY include/ DESTINATION include)

# jrl_print_dependencies_summary()
jrl_print_options_summary()
