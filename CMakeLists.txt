

#add_subdirectory(controllers)
#add_subdirectory(BTSimple)
#add_subdirectory(behaviors)

set(bt_footbot_simple_controller_IMPLEMENTATIONS
  controllers/simple/bt_footbot_simple_controller.cpp
  controllers/simple/bt_footbot_simple_root_behavior.cpp
  BTSimple/swarmanoid/footbot/ci_footbot_state.cpp
  BTSimple/swarmanoid/ci_swarmanoid_robot_state.cpp
  BTSimple/ci_robot_state.cpp
  behaviors/bt_footbot_randomwalk.cpp
  behaviors/bt_footbot_motion_control.cpp
  behaviors/bt_footbot_go_to_led.cpp
  behaviors/bt_footbot_phototaxis.cpp
  behaviors/bt_footbot_obstacle_avoidance.cpp
  #common/footbot_constants.cpp
)

add_library(bt_footbot_simple MODULE ${bt_footbot_simple_controller_IMPLEMENTATIONS})

target_link_libraries(bt_footbot_simple
  argos3core_simulator
  argos3plugin_simulator_actuators
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_sensors
)

