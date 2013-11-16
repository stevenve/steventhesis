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
 * @file common/control_interface/behavioral_toolkit/swarmanoid/ci_swarmanoid_robot_state.h
 *
 * @brief This file provides the definition for the behavioral toolkit state of the swarmanoid robots.
 * It is intended to provide all the functionalities that are shared by the swarmanoid robots, such
 * as the range and bearing support
 *
 * @author Giovanni Pini  - <gpini@iridia.ulb.ac.be>
 */

#ifndef CI_SWARMANOID_ROBOT_STATE_H
#define CI_SWARMANOID_ROBOT_STATE_H

namespace argos {
    class CCI_SwarmanoidRobotState;
}

#include "../ci_robot_state.h"
#include "../utility_classes/helper.h"
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
//#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>

namespace argos {

    class CCI_SwarmanoidRobotState : public CCI_RobotState {

        public:

            /**
             * Constructor.
             *
             * @param pc_robot Pointer to the robot object
             *
             */
            CCI_SwarmanoidRobotState(CCI_Controller* pc_controller) :
                CCI_RobotState(pc_controller),

                /** Initialize all the sensors*/
                m_pcRABSensor              (NULL),

                /** Initialize all sensors booleans */
                m_bIsUsingRABSensor        (false),

                /** Initialize all the actuators */
                m_pcRABActuator            (NULL),

                /** Initialize all actuators booleans */
                m_bIsUsingRABActuator      (false) {
                ::memset(&m_tRABPacketDataToSend, 0, sizeof(CByteArray));
            }

            /**
             *
             * Copy constructor.
             *
             **/
            CCI_SwarmanoidRobotState(const CCI_SwarmanoidRobotState& c_state) : CCI_RobotState(c_state) {
                *this = c_state;
            }

            /**
             *
             * = operator overriding. Used with the same semantic of the copy constructor
             *
             **/
            CCI_SwarmanoidRobotState& operator=(const CCI_SwarmanoidRobotState& c_state) {

                if (this != &c_state) {

                    CCI_RobotState::operator=(c_state);

                    /** Copy the references to the sensors */
                    m_pcRABSensor     = c_state.m_pcRABSensor;

                    /** Copy the sensors booleans */
                    m_bIsUsingRABSensor     = c_state.m_bIsUsingRABSensor;

                    /** Copy the references to the actuators */
                    m_pcRABActuator = c_state.m_pcRABActuator;

                    /** Copy the actuators booleans */
                    m_bIsUsingRABActuator = c_state.m_bIsUsingRABActuator;

                    /** Copy the actuated commands */
                   m_tRABPacketDataToSend = c_state.m_tRABPacketDataToSend;

                }

                return *this;
            }

            /**
             *
             * Destructor.
             *
             **/
            virtual ~CCI_SwarmanoidRobotState() {
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
            virtual void ReadState();

            /**
             *
             * @brief Apply the actuator commands with the robot state ones.
             *
             **/
            virtual void ApplyState();

            /////////////////////////////////////////////////////////////////////////////////////////////////
                        //   SENSORS GETTER METHODS, SENSOR READINGS
                        /////////////////////////////////////////////////////////////////////////////////////////////////

                        /**
                         *
                         * @brief Returns the id of the last received RAB packets.
                         *
                         * @return RAB map of last received packets.
                         * @see CCI_RangeAndBearingSensor
                         *
                         **/
                        virtual inline const CCI_RangeAndBearingSensor::TReadings GetRABLastReceivedPackets() {
                            CHECK_IS_SENSOR_USED_HELPER(RAB_SENSOR_XML_NAME,m_bIsUsingRABSensor,"GetRABLastReceivedPackets");
                            return m_pcRABSensor->GetReadings();
                        }

                        /////////////////////////////////////////////////////////////////////////////////////////////////
                        //   ACTUATORS SETTERS METHODS
                        /////////////////////////////////////////////////////////////////////////////////////////////////

                        /**
                         *
                         * @brief Sets the data to be send through range and bearing
                         * The data represents the 10 bytes RAB packet payload
                         * @param t_data RAB data to send
                         *
                         * @see CCI_RangeAndBearingSensor
                         *
                         **/
                        virtual inline void SetRABPacketDataToSend( CByteArray& t_data) {
                            CHECK_IS_ACTUATOR_USED_HELPER(RAB_ACTUATOR_XML_NAME,m_bIsUsingRABActuator,"SetRABPacketDataToSend");
                            ::memcpy(&m_tRABPacketDataToSend, &t_data, sizeof(CByteArray));
                        }

                        /**
                         *
                         * @brief Clears the data to be send by RAB
                         * The method should be called at the beginning of the control step by the root behavior
                         *
                         * @see CCI_RangeAndBearingSensor
                         *
                         **/
                        virtual inline void ClearRABPacketData() {
                            CHECK_IS_ACTUATOR_USED_HELPER(RAB_ACTUATOR_XML_NAME,m_bIsUsingRABActuator,"ClearRABPacketData");
                            ::memset(&m_tRABPacketDataToSend, 0, sizeof(CByteArray));
                        }

                        /////////////////////////////////////////////////////////////////////////////////////////////////
                        //   ACTUATORS GETTERS METHODS
                        /////////////////////////////////////////////////////////////////////////////////////////////////

                        /**
                         *
                         * @brief Gets the current RAB data to be send
                         *
                         * @return payload that will be set for the next RAB packet to send
                         * @see CCI_RangeAndBearingSensor
                         *
                         **/
                        virtual inline const CByteArray& GetRABPacketDataToSend() {
                            CHECK_IS_ACTUATOR_USED_HELPER(RAB_ACTUATOR_XML_NAME,m_bIsUsingRABActuator,"GetRABPacketDataToSend");
                            return m_tRABPacketDataToSend;
                        }

                    protected:

                        /////////////////////////////////////////////////////////////////////////////////////////////////
                        //    SENSORS
                        /////////////////////////////////////////////////////////////////////////////////////////////////

                        /** References to the sensors */
                        CCI_RangeAndBearingSensor* m_pcRABSensor;

                        /** Sensors booleans */
                        bool m_bIsUsingRABSensor;

                        /* XML sensor names */
                        static const std::string RAB_SENSOR_XML_NAME;

                        /////////////////////////////////////////////////////////////////////////////////////////////////
                        //    ACTUATORS
                        /////////////////////////////////////////////////////////////////////////////////////////////////

                        /** References to the actuators */
                        CCI_RangeAndBearingActuator* m_pcRABActuator;

                        /** Actuators booleans */
                        bool m_bIsUsingRABActuator;

                        /** Actuators commands variables */
                        CByteArray m_tRABPacketDataToSend;

                        /* XML actuator names */
                        static const std::string RAB_ACTUATOR_XML_NAME;
    };

}

#endif
