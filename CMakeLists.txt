

SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} --no-undefined" )
#add_subdirectory(BTSimple)
#add_subdirectory(behaviors)

set(bt_footbot_simple_controller_IMPLEMENTATIONS
  BTSimple/swarmanoid/footbot/ci_footbot_state.cpp
  BTSimple/swarmanoid/ci_swarmanoid_robot_state.cpp
  BTSimple/ci_robot_state.cpp
  behaviors/bt_footbot_randomwalk.cpp
  behaviors/bt_footbot_motion_control.cpp
  behaviors/bt_footbot_go_to_led.cpp
  behaviors/bt_footbot_phototaxis.cpp
  behaviors/bt_footbot_obstacle_avoidance.cpp
  behaviors/bt_footbot_observe_ground.cpp
  behaviors/bt_footbot_control_leds.cpp
  behaviors/bt_footbot_odometry.cpp
  behaviors/bt_footbot_RAB_signal.cpp
#  behaviors/bt_footbot_beacon.cpp
)

add_library(bt_footbot_base SHARED ${bt_footbot_simple_controller_IMPLEMENTATIONS})

target_link_libraries(bt_footbot_base
  argos3core_simulator
  argos3plugin_simulator_actuators
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_sensors
)

add_subdirectory(loop_functions)
add_subdirectory(controllers)
