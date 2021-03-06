/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**
 * @file common/control_interface/behavioral_toolkit/swarmanoid/ci_swarmanoid_robot_state.cpp
 *
 * @brief This file provides the implementation for the behavioral toolkit state of the swarmanoid robots.
 * It is intended to provide all the functionalities that are shared by the swarmanoid robots, such
 * as the range and bearing support
 *
 * @author Giovanni Pini  - <gpini@iridia.ulb.ac.be>
 */

#include "ci_swarmanoid_robot_state.h"

namespace argos{

/****************************************/
/****************************************/

const std::string CCI_SwarmanoidRobotState::RAB_SENSOR_XML_NAME     = "range_and_bearing";
const std::string CCI_SwarmanoidRobotState::RAB_ACTUATOR_XML_NAME   = "range_and_bearing";
const std::string CCI_SwarmanoidRobotState::LEDS_ACTUATOR_XML_NAME             = "leds";
const std::string CCI_SwarmanoidRobotState::STEERING_SENSOR_XML_NAME             = "differential_steering";

//Actuator numbers
const UInt8  CCI_SwarmanoidRobotState::NUM_LEDS = 13;
/****************************************/
/****************************************/

void CCI_SwarmanoidRobotState::Init(){

	m_tRABPacketDataToSend = CByteArray(10);

	CCI_RobotState::Init();

	CCI_Sensor::TMap mapSensors = m_pcController->GetAllSensors();
	CCI_Sensor::TMap::const_iterator itSensors;

	SENSOR_INIT_HELPER(RAB_SENSOR_XML_NAME, CCI_RangeAndBearingSensor, m_pcRABSensor, m_bIsUsingRABSensor);
	SENSOR_INIT_HELPER(STEERING_SENSOR_XML_NAME, CCI_DifferentialSteeringSensor, m_pcSteeringSensor, m_bIsUsingSteeringSensor);

	///////////////////////////////////////////////////////////////////
	//   INITIALIZE ALL THE ACTUATORS DECLARED IN THE XML CONFIGURATION
	///////////////////////////////////////////////////////////////////

	CCI_Actuator::TMap mapActuators = m_pcController->GetAllActuators();
	CCI_Actuator::TMap::const_iterator itActuators;

	ACTUATOR_INIT_HELPER(RAB_ACTUATOR_XML_NAME, CCI_RangeAndBearingActuator, m_pcRABActuator, m_bIsUsingRABActuator);
	ACTUATOR_INIT_HELPER(LEDS_ACTUATOR_XML_NAME,                CCI_LEDsActuator,            m_pcLedsActuator,               m_bIsUsingLeds);
}

/****************************************/
/****************************************/

void CCI_SwarmanoidRobotState::ApplyState(){

	CCI_RobotState::ApplyState();

	if (m_bRefreshLeds && m_bIsUsingLeds) {
		m_bRefreshLeds = false;
		m_pcLedsActuator->SetAllColors( m_tActuatedLedColors );
	}

	if (m_bIsUsingRABActuator) {
		m_pcRABActuator->SetData(m_tRABPacketDataToSend);
	}
}

/****************************************/
/****************************************/

void CCI_SwarmanoidRobotState::ReadState(){
}

/****************************************/
/****************************************/

}
