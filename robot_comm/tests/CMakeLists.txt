find_package(GTest REQUIRED)
include(GoogleTest)

set(TEST_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config.in.h ${CMAKE_CURRENT_BINARY_DIR}/config.h
  @ONLY
)

set(TEST_MAIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/test_main.cpp)

function(add_mc_test TEST_NAME)
  add_executable(${TEST_NAME} ${TEST_MAIN_SRC} ${ARGN})

  target_link_libraries(${TEST_NAME} PRIVATE robot_comm GTest::gtest)

  target_include_directories(
    ${TEST_NAME} PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  )
  set_target_properties(
    ${TEST_NAME} PROPERTIES BUILD_RPATH "$ORIGIN/../lib" INSTALL_RPATH
                                                         "$ORIGIN/../lib"
  )

  gtest_discover_tests(${TEST_NAME} LABELS "communication" TIMEOUT 10)
endfunction()

add_mc_test(test_config test_config.cpp)
add_mc_test(test_server test_server.cpp)
add_mc_test(test_multiple test_multiple.cpp)
# add_mc_test(test_query_reply test_query_reply.cpp)

# add_mc_test(test_shm test_shm.cpp)
