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
 * @file common/control_interface/behavioral_toolkit/swarmanoid/footbot/ci_footbot_state.cpp
 *
 * @brief This file provides the control interface behavioral toolkit implementation for a foot-bot state.
 *
 * @author Eliseo Ferrante - <eferrant@ulb.ac.be>
 * @author Nithin Mathews - <nmathews@ulb.ac.be>
 * @author Giovanni Pini - <gpini@iridia.ulb.ac.be>
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 * @author Arne Brutschy - <arne.brutschy@ulb.ac.be>
 */

#include "ci_footbot_state.h"

namespace argos {

	//XML sensors names
	const std::string CCI_FootBotState::BASE_GROUND_SENSOR_XML_NAME            = "footbot_base_ground";
	const std::string CCI_FootBotState::OMNIDIRECTIONAL_CAMERA_SENSOR_XML_NAME = "colored_blob_omnidirectional_camera";
    const std::string CCI_FootBotState::LIGHT_SENSOR_XML_NAME                  = "footbot_light";
    const std::string CCI_FootBotState::PROXIMITY_SENSORS_XML_NAME             = "footbot_proximity";
    const std::string CCI_FootBotState::MOTOR_GROUND_SENSOR_XML_NAME           = "footbot_motor_ground";

    //XML actuators names
    const std::string CCI_FootBotState::WHEELS_ACTUATOR_XML_NAME           = "differential_steering";


    /****************************************/
    /****************************************/

    void CCI_FootBotState::Init()
    {

        CCI_SwarmanoidRobotState::Init();

        ///////////////////////////////////////////////////////////////////
        //   INITIALIZE ALL THE SENSORS DECLARED IN THE XML CONFIGURATION
        ///////////////////////////////////////////////////////////////////

        CCI_Sensor::TMap mapSensors = m_pcController->GetAllSensors();
        CCI_Sensor::TMap::const_iterator itSensors;

        SENSOR_INIT_HELPER(BASE_GROUND_SENSOR_XML_NAME,             CCI_FootBotBaseGroundSensor,            m_pcBaseGroundSensor,               m_bIsUsingBaseGroundSensor);
        SENSOR_INIT_HELPER(OMNIDIRECTIONAL_CAMERA_SENSOR_XML_NAME,  CCI_ColoredBlobOmnidirectionalCameraSensor, m_pcOmnidirectionalCameraSensor,    m_bIsUsingOmnidirectionalCameraSensor);
        SENSOR_INIT_HELPER(LIGHT_SENSOR_XML_NAME,                   CCI_FootBotLightSensor,                 m_pcLightSensor,                    m_bIsUsingLightSensor);
        SENSOR_INIT_HELPER(PROXIMITY_SENSORS_XML_NAME,              CCI_FootBotProximitySensor,             m_pcProximitySensor,                m_bIsUsingProximitySensor);
        SENSOR_INIT_HELPER(MOTOR_GROUND_SENSOR_XML_NAME,            CCI_FootBotMotorGroundSensor,           m_pcMotorGroundSensor,              m_bIsUsingMotorGroundSensor);

        ///////////////////////////////////////////////////////////////////
        //   INITIALIZE SENSOR RELATED VARIABLES
        ///////////////////////////////////////////////////////////////////

        if (m_bIsUsingOmnidirectionalCameraSensor)
            	 m_pcOmnidirectionalCameraSensor->Enable();
        if (m_bIsUsingLightSensor)
                    m_tLightSensorReadings = &m_pcLightSensor->GetReadings();

        ///////////////////////////////////////////////////////////////////
        //   INITIALIZE ALL THE ACTUATORS DECLARED IN THE XML CONFIGURATION
        ///////////////////////////////////////////////////////////////////

        CCI_Actuator::TMap mapActuators = m_pcController->GetAllActuators();
        CCI_Actuator::TMap::const_iterator itActuators;

        ACTUATOR_INIT_HELPER(WHEELS_ACTUATOR_XML_NAME,              CCI_DifferentialSteeringActuator,          m_pcWheelsActuator,             m_bIsUsingWheelsActuator);
        }

    /****************************************/
    /****************************************/

    void CCI_FootBotState::ApplyState()
    {
        CCI_SwarmanoidRobotState::ApplyState();

        if (m_bRefreshWheelsActuator && m_bIsUsingWheelsActuator) {
            m_bRefreshWheelsActuator = false;
            m_pcWheelsActuator->SetLinearVelocity(m_fActuatedLeftWheelSpeed,
                    m_fActuatedRightWheelSpeed);
        }
    }

    /****************************************/
    /****************************************/

}
