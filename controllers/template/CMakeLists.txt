set(bt_footbot_template_controller_HEADERS
  bt_footbot_template_controller.h
  bt_footbot_template_root_behavior.h
)

set(bt_footbot_template_controller_IMPLEMENTATIONS
  bt_footbot_template_controller.cpp
  bt_footbot_template_root_behavior.cpp
)

if( ARGOS_COMPILE_REAL_SWARMANOID )

  link_directories(
    ${CMAKE_SOURCE_DIR}/../../build/common/swarmanoid/control_interface
    ${CMAKE_SOURCE_DIR}/../../build/common/swarmanoid/utility
    ${CMAKE_SOURCE_DIR}/../../build/real_robot/swarmanoid/common
    ${CMAKE_SOURCE_DIR}/../../build/real_robot/swarmanoid/footbot
    ${CMAKE_SOURCE_DIR}/../../build/real_robot/swarmanoid/vision_robot/filtering
    ${CMAKE_SOURCE_DIR}/../../real_robot/swarmanoid/opencv/lib
    ${CMAKE_BINARY_DIR}/behaviors/
  )
  add_executable(bt_footbot_template_controller
    ${bt_footbot_template_controller_IMPLEMENTATIONS}
    ${CMAKE_SOURCE_DIR}/main_functions/real_footbot_main.cpp
  )
  target_link_libraries(argos2_real_robot_swarmanoid_footbot)

else(  ARGOS_COMPILE_REAL_SWARMANOID)

	link_directories(
		${CMAKE_BINARY_DIR}/common/
		${CMAKE_BINARY_DIR}/behaviors/
	)

  add_library(bt_footbot_template_controller MODULE ${bt_footbot_template_controller_IMPLEMENTATIONS})

endif(  ARGOS_COMPILE_REAL_SWARMANOID )

target_link_libraries(bt_footbot_template_controller
	eliseofe_common_functions
  eliseofe_footbot_behaviors
)



