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
/usr/lib/argos3/libargos3core_simulator.so

/usr/lib/argos3/libargos3plugin_simulator_actuators.so
  /usr/lib/argos3/libargos3plugin_simulator_dynamics2d.so
/usr/lib/argos3/libargos3plugin_simulator_entities.so
/usr/lib/argos3/libargos3plugin_simulator_footbot.so
/usr/lib/argos3/libargos3plugin_simulator_genericrobot.so
/usr/lib/argos3/libargos3plugin_simulator_media.so
/usr/lib/argos3/libargos3plugin_simulator_sensors.so)

if(ARGOS_COMPILE_QTOPENGL)
target_link_libraries(simple_loop_functions /usr/lib/argos3/libargos3plugin_simulator_qtopengl.so)
endif(ARGOS_COMPILE_QTOPENGL)
