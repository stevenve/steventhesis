
add_library(bt_footbot_behaviors MODULE bt_footbot_randomwalk.cpp)

target_link_libraries(bt_footbot_behaviors
  argos3core_simulator
  argos3plugin_simulator_actuators
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_sensors
)








