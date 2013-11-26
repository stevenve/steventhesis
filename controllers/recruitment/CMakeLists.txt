
set(bt_footbot_simple_controller_IMPLEMENTATIONS
  bt_footbot_recruitment_controller.cpp
  bt_footbot_recruitment_root_behavior.cpp
)

#link_directories(
#		${CMAKE_SOURCE_DIR}/BTSimple/behaviors/
#	)

add_library(bt_footbot_recruitment SHARED bt_footbot_recruitment_controller.cpp bt_footbot_recruitment_root_behavior.cpp)

target_link_libraries(bt_footbot_recruitment
  bt_footbot_base
#  behavioral_toolkit
/usr/lib/argos3/libargos3core_simulator.so
  /usr/lib/argos3/libargos3plugin_simulator_actuators.so
/usr/lib/argos3/libargos3plugin_simulator_footbot.so
/usr/lib/argos3/libargos3plugin_simulator_genericrobot.so
/usr/lib/argos3/libargos3plugin_simulator_sensors.so
)








