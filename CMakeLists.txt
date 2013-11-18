


#add_subdirectory(BTSimple)
#add_subdirectory(behaviors)

set(bt_footbot_simple_controller_IMPLEMENTATIONS
  controllers/randomforaging/bt_footbot_randomforaging_controller.cpp
  controllers/randomforaging/bt_footbot_randomforaging_root_behavior.cpp
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
  #common/footbot_constants.cpp
  loop_functions/recruitment_loop_functions.cpp
)

add_library(bt_footbot_simple SHARED ${bt_footbot_simple_controller_IMPLEMENTATIONS})

target_link_libraries(bt_footbot_simple
  argos3core_simulator
  argos3plugin_simulator_actuators
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_sensors
)

add_subdirectory(loop_functions)
