link_directories(${CMAKE_BINARY_DIR}/problem/controllers/randomforaging)
set(simple_loop_functions_SOURCES simple_loop_functions.cpp)

include(ARGoSCheckQTOpenGL)
if(ARGOS_COMPILE_QTOPENGL)
  set(simple_loop_functions_SOURCES
    ${simple_loop_functions_SOURCES}
    simple_qt_user_functions.cpp)
endif(ARGOS_COMPILE_QTOPENGL)

add_library(simple_loop_functions MODULE ${simple_loop_functions_SOURCES})
target_link_libraries(simple_loop_functions
  bt_footbot_randomforaging
  bt_footbot_base
  argos3core_simulator
  argos3plugin_simulator_actuators
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_sensors
argos3plugin_simulator_media
argos3plugin_simulator_dynamics2d
argos3plugin_simulator_entities)

if(ARGOS_COMPILE_QTOPENGL)
target_link_libraries(simple_loop_functions /usr/lib/argos3/libargos3plugin_simulator_qtopengl.so)
endif(ARGOS_COMPILE_QTOPENGL)
