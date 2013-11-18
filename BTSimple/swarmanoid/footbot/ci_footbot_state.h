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
 * @file common/control_interface/behavioral_toolkit/swarmanoid/footbot/ci_footbot_state.h
 *
 * @brief This file provides the control interface behavioral toolkit definition for a foot-bot state.
 * Check file ci_robot_state.h for the full explanation.
 *
 * @author Eliseo Ferrante - <eferrant@ulb.ac.be>
 * @author Nithin Mathews - <nmathews@ulb.ac.be>
 * @author Giovanni Pini - <gpini@iridia.ulb.ac.be>
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 * @author Arne Brutschy - <arne.brutschy@ulb.ac.be>
 */

#ifndef CI_FOOTBOT_STATE_H
#define CI_FOOTBOT_STATE_H

namespace argos {
    class CCI_FootBotState;
}

#include "../ci_swarmanoid_robot_state.h"
#include <argos3/core/utility/math/angles.h>

/* Sensor includes */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_base_ground_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>

/* Actuator includes */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>

namespace argos {

    class CCI_FootBotState: public CCI_SwarmanoidRobotState {

    public:

        /////////////////////////////////////////////////////////////////////////////////////////////////
        //    CONSTRUCTORS
        /////////////////////////////////////////////////////////////////////////////////////////////////

        /**
         * Constructor.
         *
         * @param pc_robot Pointer to the robot object
         *
         */
        CCI_FootBotState(CCI_Controller* pc_controller) :
            CCI_SwarmanoidRobotState(pc_controller),

            /** Initialize all the sensors */
            m_pcBaseGroundSensor            (NULL),
            m_pcOmnidirectionalCameraSensor(NULL),
            m_pcLightSensor                 (NULL),
            m_pcProximitySensor             (NULL),
            m_pcMotorGroundSensor           (NULL),

            /** Initialize all sensors booleans */
            m_bIsUsingBaseGroundSensor              (false),
            m_bIsUsingOmnidirectionalCameraSensor           (false),
            m_bIsUsingLightSensor                   (false),
            m_bIsUsingProximitySensor               (false),
            m_tLightSensorReadings			(NULL),
            m_bIsUsingMotorGroundSensor             (false),

            /** Initialize all the actuators */
            m_pcWheelsActuator          (NULL),

            /** Initialize all actuators booleans */
            m_bIsUsingWheelsActuator          (false),

            /** Initialize all actuators refresh booleans */
            m_bRefreshWheelsActuator               (false),

            /** Initialize the actuated values */
            m_fActuatedLeftWheelSpeed         (0.0),
            m_fActuatedRightWheelSpeed        (0.0){
        }

        /**
         *
         * Copy Constructor.
         *
         */
        CCI_FootBotState(const CCI_FootBotState& c_footbot_state) : CCI_SwarmanoidRobotState(c_footbot_state)
        {
            *this = c_footbot_state;
        }

        /**
         *
         * = operator overriding. Used with the same semantic of the copy constructor
         *
         */
        CCI_FootBotState& operator=(const CCI_FootBotState& c_footbot_state)
        {

            if (this != &c_footbot_state) {
                CCI_SwarmanoidRobotState::operator=(c_footbot_state);

            	/** Copy the references to the sensors */
                m_pcBaseGroundSensor            = c_footbot_state.m_pcBaseGroundSensor;
                m_pcOmnidirectionalCameraSensor         = c_footbot_state.m_pcOmnidirectionalCameraSensor;
            	m_pcLightSensor                 = c_footbot_state.m_pcLightSensor;
            	m_pcProximitySensor             = c_footbot_state.m_pcProximitySensor;
            	m_pcMotorGroundSensor           = c_footbot_state.m_pcMotorGroundSensor;

            	/** Copy the sensors booleans */
            	m_bIsUsingBaseGroundSensor              = c_footbot_state.m_bIsUsingBaseGroundSensor;
            	m_bIsUsingOmnidirectionalCameraSensor           = c_footbot_state.m_bIsUsingOmnidirectionalCameraSensor;
            	m_bIsUsingLightSensor                   = c_footbot_state.m_bIsUsingLightSensor;
            	m_bIsUsingProximitySensor               = c_footbot_state.m_bIsUsingProximitySensor;
            	m_bIsUsingMotorGroundSensor             = c_footbot_state.m_bIsUsingMotorGroundSensor;

            	/** Copy other sensor related variables */
            	m_tLightSensorReadings    = c_footbot_state.m_tLightSensorReadings;

                /** Copy the references to the actuators */
                m_pcWheelsActuator          = c_footbot_state.m_pcWheelsActuator;

                /** Copy the actuators booleans */
                m_bIsUsingWheelsActuator          = c_footbot_state.m_bIsUsingWheelsActuator;

                /** Copy the actuators refresh booleans */
                m_bRefreshWheelsActuator               = c_footbot_state.m_bRefreshWheelsActuator;

                /** Copy the actuated commands */
                m_fActuatedLeftWheelSpeed             = c_footbot_state.m_fActuatedLeftWheelSpeed;
                m_fActuatedRightWheelSpeed            = c_footbot_state.m_fActuatedRightWheelSpeed;

                /** Copy the actuated commands */
                m_fActuatedLeftWheelSpeed             = c_footbot_state.m_fActuatedLeftWheelSpeed;
                m_fActuatedRightWheelSpeed            = c_footbot_state.m_fActuatedRightWheelSpeed;
            }

            return *this;
        }

        /**
         *
         * Destructor.
         *
         **/
        virtual ~CCI_FootBotState()
        {
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////
        //   MISC FUNCTIONS
        /////////////////////////////////////////////////////////////////////////////////////////////////

        /**
         *
         * @brief Initializes all the sensors and actuators declared in the XML.
         *
         **/
        virtual void Init();

        /**
         *
         * @brief Reads sensor data and sets the robot state accordingly.
         *
         **/
        virtual void ReadState() {}

        /**
         *
         * @brief Applies the actuator commands with the robot state ones.
         *
         **/
        virtual void ApplyState();

        //////////////////////////////////////////////////////////////////////////////////////////////
        //   SENSORS GETTER METHODS, SENSOR READINGS												//
        //////////////////////////////////////////////////////////////////////////////////////////////

        /**
                 *
                 * @brief Returns the motor ground sensors readings
                 * The 4 motor ground sensors are positioned underneath the foot-bot, between the 2 tracks
                 *
                 * @return the motor ground sensors readings
                 * @see CCI_FootBotMotorGroundSensor
                 *
                 **/
                inline virtual const CCI_FootBotMotorGroundSensor::TReadings& GetMotorGroundSensorReadings()
                {
                    CHECK_IS_SENSOR_USED_HELPER(MOTOR_GROUND_SENSOR_XML_NAME, m_bIsUsingMotorGroundSensor, "GetMotorGroundSensorReadings");
                    return m_pcMotorGroundSensor->GetReadings();
                }

        /**
        *
        * @brief Returns the base ground sensors readings
        * The 8 base ground sensors are positioned underneath the LED ring, where the proximity sensors are.
        *
        * @return the base ground sensors readings
        * @see CCI_FootBotBaseGroundSensor
        *
        **/
        inline virtual const CCI_FootBotBaseGroundSensor::TReadings& GetBaseGroundSensorReadings()
        {
        	CHECK_IS_SENSOR_USED_HELPER(BASE_GROUND_SENSOR_XML_NAME, m_bIsUsingBaseGroundSensor, "GetBaseGroundSensorReadings");
        	return m_pcBaseGroundSensor->GetReadings();
        }


        virtual inline const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& GetOmnidirectionalCameraReadings(){
                    CHECK_IS_SENSOR_USED_HELPER(OMNIDIRECTIONAL_CAMERA_SENSOR_XML_NAME, m_bIsUsingOmnidirectionalCameraSensor, "GetOmnidirectionalCameraBlobs");
                    return m_pcOmnidirectionalCameraSensor->GetReadings();
        }

        /**
        * @brief Returns the light sensors readings and angles
        *
        * @return the light sensors readings and angles
        * @see CCI_FootBotLightSensor
        *
        **/
        inline virtual const CCI_FootBotLightSensor::TReadings& GetLightSensorReadings()
        {
        	CHECK_IS_SENSOR_USED_HELPER(LIGHT_SENSOR_XML_NAME, m_bIsUsingLightSensor, "GetLightSensorReadings");
        	return m_pcLightSensor->GetReadings();
        }

        /**
        *
        * @brief Returns the proximity sensors readings
        *
        * @return the proximity sensors readings
        * @see CCI_FootBotProximitySensor
        *
        **/
        inline virtual const CCI_FootBotProximitySensor::TReadings& GetProximitySensorReadings()
        {
        	CHECK_IS_SENSOR_USED_HELPER(PROXIMITY_SENSORS_XML_NAME, m_bIsUsingProximitySensor, "GetProximitySensorReadings");
        	return m_pcProximitySensor->GetReadings();
        }

        //////////////////////////////////////////////////////////////////////////////////////////////
        //   ACTUATORS SETTERS METHODS																//
        //////////////////////////////////////////////////////////////////////////////////////////////

        /**
         *
         * @brief Sets the wheels speeds
         * speeds are linear and expressed in cm/s
         * notice that the actual speeds will be set in the ApplyState() method
         *
         * @param f_left_velocity linear speed for the left wheel
         * @param f_right_velocity linear speed for the right wheel
         * @see CCI_FootBotWheelSpeedSensor
         * @see CCI_FootBotWheelsActuator
         *
         **/
        inline virtual void SetWheelsLinearVelocity(Real f_left_velocity, Real f_right_velocity)
        {
            CHECK_IS_ACTUATOR_USED_HELPER(WHEELS_ACTUATOR_XML_NAME, m_bIsUsingWheelsActuator, "SetWheelsLinearVelocity");
            m_bRefreshWheelsActuator = true;
            m_fActuatedLeftWheelSpeed  = f_left_velocity;
            m_fActuatedRightWheelSpeed = f_right_velocity;
        }



        ///////////////////////////////////////////////////////////////////////////////////////////
        //   ACTUATORS GETTERS METHOD
        //   WARNING: many of these methods return the actuated commands that are going to be send.
        //            Therefore the values are not measures of the real state of the robot.
        ///////////////////////////////////////////////////////////////////////////////////////////

        /**
         *
         * @brief Returns the current target left wheel speed
         * notice that the actual speed could be different
         * speed is expressed in cm/s
         *
         * @return target left wheel speed
         * @see CCI_FootBotWheelSpeedSensor
         * @see CCI_FootBotWheelsActuator
         *
         **/
        inline virtual Real GetActuatedLeftWheelVelocity()
        {
            CHECK_IS_ACTUATOR_USED_HELPER(WHEELS_ACTUATOR_XML_NAME, m_bIsUsingWheelsActuator, "GetActuatedLeftWheelVelocity");
            return m_fActuatedLeftWheelSpeed;
        }

        /**
         *
         * @brief Returns the current target right wheel speed
         * notice that the actual speed could be different
         * speed is expressed in cm/s
         *
         * @return target right wheel speed
         * @see CCI_FootBotWheelSpeedSensor
         * @see CCI_FootBotWheelsActuator
         *
         **/
        inline virtual Real GetActuatedRightWheelVelocity()
        {
            CHECK_IS_ACTUATOR_USED_HELPER(WHEELS_ACTUATOR_XML_NAME, m_bIsUsingWheelsActuator, "GetActuatedRightWheelVelocity");
            return m_fActuatedRightWheelSpeed;
        }


    protected:

        /////////////
        //    SENSORS
        /////////////

        /** References to the sensors */
        CCI_FootBotBaseGroundSensor*            m_pcBaseGroundSensor;
        CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcOmnidirectionalCameraSensor;
        CCI_FootBotLightSensor*                 m_pcLightSensor;
        CCI_FootBotProximitySensor*             m_pcProximitySensor;
        CCI_FootBotMotorGroundSensor*           m_pcMotorGroundSensor;

        /** Sensors booleans */
        bool m_bIsUsingBaseGroundSensor;
        bool m_bIsUsingOmnidirectionalCameraSensor;
        bool m_bIsUsingLightSensor;
        bool m_bIsUsingProximitySensor;
        bool m_bIsUsingMotorGroundSensor;

        /** Variables related to sensors */
        const CCI_FootBotLightSensor::TReadings*    m_tLightSensorReadings;

        /* XML sensor names */
        static const std::string BASE_GROUND_SENSOR_XML_NAME;
        static const std::string OMNIDIRECTIONAL_CAMERA_SENSOR_XML_NAME;
        static const std::string LIGHT_SENSOR_XML_NAME;
        static const std::string PROXIMITY_SENSORS_XML_NAME;
        static const std::string MOTOR_GROUND_SENSOR_XML_NAME;

        ///////////////
        //    ACTUATORS
        ///////////////

        /** References to the actuators */
        CCI_DifferentialSteeringActuator*          m_pcWheelsActuator;

        /** Actuators booleans */
        bool m_bIsUsingWheelsActuator;

        /** Actuator refresh booleans */
        bool m_bRefreshWheelsActuator;

        /** Actuators commands variables */
        Real                                    m_fActuatedLeftWheelSpeed;
        Real                                    m_fActuatedRightWheelSpeed;

        /* XML actuator names */
        static const std::string WHEELS_ACTUATOR_XML_NAME;

    };

}

#endif
