find_package(GTest REQUIRED)
find_package(robot_comm QUIET)
include(GoogleTest)

function(add_ri_test TEST_NAME)
  add_executable(${TEST_NAME} ${ARGN})
  target_link_libraries(${TEST_NAME} PRIVATE GTest::gtest_main)

  if(robot_comm_FOUND)
    target_link_libraries(${TEST_NAME} PRIVATE robot_comm::robot_comm)
  else()
    # Monorepo build: robot_comm added via add_subdirectory.
    target_link_libraries(${TEST_NAME} PRIVATE robot_comm)
  endif()

  gtest_discover_tests(${TEST_NAME})
endfunction()

add_ri_test(test_init_handshake test_init_handshake.cpp)
add_ri_test(test_pubsub_loop test_pubsub_loop.cpp)

# Tiny driver plugin living in the build tree, outside any install prefix.
# A second copy with the same class name, in another directory, checks that
# the first directory in the search order wins. Each gets its own directory so
# the loader does not scan the whole lib/ dir.
add_library(search_path_test_driver MODULE test_driver.cpp)
target_link_libraries(search_path_test_driver PRIVATE robot_driver_api)
jrl_target_set_output_directory(
  search_path_test_driver OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/driver_a
)

add_library(search_path_test_driver_b MODULE test_driver.cpp)
target_link_libraries(search_path_test_driver_b PRIVATE robot_driver_api)
target_compile_definitions(
  search_path_test_driver_b PRIVATE TEST_DRIVER_Q0=10.0
)
jrl_target_set_output_directory(
  search_path_test_driver_b OUTPUT_DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/driver_b
)

add_ri_test(test_driver_search_paths test_driver_search_paths.cpp)
target_link_libraries(
  test_driver_search_paths PRIVATE robot_interface_lib mc_rtc::mc_rtc_loader
)
target_compile_definitions(
  test_driver_search_paths
  PRIVATE TEST_DRIVER_DIR="$<TARGET_FILE_DIR:search_path_test_driver>"
          TEST_DRIVER_B_DIR="$<TARGET_FILE_DIR:search_path_test_driver_b>"
)
add_dependencies(
  test_driver_search_paths search_path_test_driver search_path_test_driver_b
)

add_ri_test(test_link_loss test_link_loss.cpp)
# RobotInterface.h instantiates mc_rtc's loader, which robot_interface_lib does not
# propagate.
target_link_libraries(
  test_link_loss PRIVATE robot_interface_lib mc_rtc::mc_rtc_loader
)
