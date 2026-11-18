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
add_library(search_path_test_driver MODULE test_driver.cpp)
target_link_libraries(search_path_test_driver PRIVATE robot_driver_api)

add_ri_test(test_driver_search_paths test_driver_search_paths.cpp)
target_link_libraries(
  test_driver_search_paths PRIVATE robot_interface_lib mc_rtc::mc_rtc_loader
)
target_compile_definitions(
  test_driver_search_paths
  PRIVATE TEST_DRIVER_DIR="$<TARGET_FILE_DIR:search_path_test_driver>"
)
add_dependencies(test_driver_search_paths search_path_test_driver)
