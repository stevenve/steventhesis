
set(bt_footbot_combined_controller_IMPLEMENTATIONS
  bt_footbot_combined_novis_controller.cpp
  bt_footbot_combined_novis_root_behavior.cpp
)

#link_directories(
#		${CMAKE_SOURCE_DIR}/BTSimple/behaviors/
#	)

add_library(bt_footbot_combined_novis SHARED bt_footbot_combined_novis_controller.cpp bt_footbot_combined_novis_root_behavior.cpp)

target_link_libraries(bt_footbot_combined_novis
  bt_footbot_base
#  behavioral_toolkit
  argos3core_simulator
  argos3plugin_simulator_actuators
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_sensors
)








