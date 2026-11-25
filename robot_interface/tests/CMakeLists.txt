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
add_ri_test(test_link_loss test_link_loss.cpp)
# RobotInterface.h instantiates mc_rtc's loader, which robot_interface_lib does not
# propagate.
target_link_libraries(
  test_link_loss PRIVATE robot_interface_lib mc_rtc::mc_rtc_loader
)
