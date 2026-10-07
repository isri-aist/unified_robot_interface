include(GNUInstallDirs)

# Robot Interface
set(INTERFACE_SRC robot_interface/RobotInterfaceFactory.cpp
                  robot_interface/FMInterfaceTemplate.cpp
)

add_library(robot_interface SHARED ${INTERFACE_SRC})
add_library(${PROJECT_NAME}::robot_interface ALIAS robot_interface)

target_link_libraries(
  robot_interface
  PUBLIC robot_controller::core robot_comm
  PRIVATE mc_rtc::mc_control
)

target_include_directories(
  robot_interface
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Robot Manager
set(ROBOT_MANAGER_SRC robot_manager/EntryPoint.cpp
                      robot_manager/RobotManager.cpp
)

add_library(robot_manager SHARED ${ROBOT_MANAGER_SRC})
add_library(${PROJECT_NAME}::robot_manager ALIAS robot_manager)

target_link_libraries(
  robot_manager PUBLIC robot_controller::core robot_comm mc_rtc::mc_control
)

target_include_directories(
  robot_manager
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../third_party
)

install(
  TARGETS robot_interface robot_manager
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Executable
add_executable(uri robot_manager/main.cpp)

target_include_directories(
  uri PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../third_party
              ${CMAKE_CURRENT_BINARY_DIR}/include
)

# Embed robot_interface/tools/create_new_driver.sh for `uri create_new_driver`.
set(CREATE_NEW_DRIVER_SCRIPT
    ${CMAKE_CURRENT_LIST_DIR}/../../robot_interface/tools/create_new_driver.sh
)
set_property(
  DIRECTORY
  APPEND
  PROPERTY CMAKE_CONFIGURE_DEPENDS ${CREATE_NEW_DRIVER_SCRIPT}
)
file(READ ${CREATE_NEW_DRIVER_SCRIPT} CREATE_NEW_DRIVER_SCRIPT_CONTENT)
file(
  WRITE ${CMAKE_CURRENT_BINARY_DIR}/CreateNewDriverScript.h.tmp
  "#pragma once\n\n// Generated from robot_interface/tools/create_new_driver.sh, do not edit.\n"
  "inline constexpr const char * CREATE_NEW_DRIVER_SCRIPT = R\"__uri_script__(${CREATE_NEW_DRIVER_SCRIPT_CONTENT})__uri_script__\";\n"
)
configure_file(
  ${CMAKE_CURRENT_BINARY_DIR}/CreateNewDriverScript.h.tmp
  ${CMAKE_CURRENT_BINARY_DIR}/include/CreateNewDriverScript.h COPYONLY
)
target_link_libraries(
  uri PRIVATE robot_manager robot_interface robot_comm robot_interface_lib
              mc_rtc::mc_rtc_loader
)

# Installed executable and libraries use the same custom installation prefix.
# Use an absolute RPATH because uri receives file capabilities.
set_target_properties(
  robot_interface robot_manager uri
  PROPERTIES
    INSTALL_RPATH
    "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR};${CMAKE_INSTALL_PREFIX}/lib"
)

set(PROGRAM_NAME uri)

set(JOB_POOL_OPTION)
if(CMAKE_VERSION VERSION_GREATER "3.14")
  set(JOB_POOL_OPTION JOB_POOL console)
endif()

install(TARGETS ${PROGRAM_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(USE_SETCAP)
  add_custom_target(
    setcap_${PROGRAM_NAME} ALL
    COMMAND ${SUDO} -S ${SETCAP} cap_sys_nice+eip $<TARGET_FILE:${PROGRAM_NAME}>
    DEPENDS ${PROGRAM_NAME} ${JOB_POOL_OPTION}
  )

  install(
    CODE
    "execute_process(
         COMMAND
           ${SUDO} -S ${SETCAP} cap_sys_nice+eip
           ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/${PROGRAM_NAME}
       )"
  )
endif()
