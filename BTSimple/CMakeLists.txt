


set(BT_SOURCES 
swarmanoid/footbot/ci_footbot_state.cpp
ci_robot_state.cpp
)


add_library(behavioral_toolkit MODULE ${BT_SOURCES})


target_link_libraries(
behavioral_toolkit
#bt_footbot_simple
  argos3core_simulator
  argos3plugin_simulator_actuators
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_sensors
)
