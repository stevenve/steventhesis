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
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

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
                m_pcLedsActuator				(NULL),
                m_pcRABActuator            (NULL),

                /** Initialize all actuators booleans */
                m_bIsUsingLeds                    (false),
                m_bIsUsingRABActuator      (false),

                /** Initialize all actuators refresh booleans */
                m_bRefreshLeds                         (false){
                ::memset(&m_tRABPacketDataToSend, 0, sizeof(CByteArray));

                /** Initialize the actuated values */
                m_tActuatedLedColors = CCI_LEDsActuator::TSettings(NUM_LEDS);
                for (UInt8 i = 0; i < NUM_LEDS; i++)
                	m_tActuatedLedColors[i] = CColor::BLACK;
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
                    m_pcLedsActuator            = c_state.m_pcLedsActuator;

                    /** Copy the actuators booleans */
                    m_bIsUsingRABActuator = c_state.m_bIsUsingRABActuator;
                    m_bIsUsingLeds                    = c_state.m_bIsUsingLeds;

                    /** Copy the actuators refresh booleans */
                    m_bRefreshLeds			= c_state.m_bRefreshLeds;

                    /** Copy the actuated commands */
                   m_tRABPacketDataToSend = c_state.m_tRABPacketDataToSend;
                   m_tActuatedLedColors                  = c_state.m_tActuatedLedColors;

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

                        /**
                                 *
                                 * @brief Sets the color for the specified LED
                                 * notice that the actual color will be set in the ApplyState() method
                                 *
                                 * @param un_index index of the LED to be set
                                 * @param c_color color to be set
                                 * @see CColor
                                 *
                                 **/
                                inline virtual void SetSingleLedColor(UInt8 un_index, const CColor& c_color)
                                {
                                    CHECK_IS_ACTUATOR_USED_HELPER(LEDS_ACTUATOR_XML_NAME, m_bIsUsingLeds, "SetSingleLedColor");
                                    m_bRefreshLeds = true;
                                    m_tActuatedLedColors[un_index] = CColor(c_color.GetRed(), c_color.GetGreen(), c_color.GetBlue(), c_color.GetAlpha());
                                }

                                /**
                                         *
                                         * @brief Sets the color for all the LEDs
                                         * notice that the actual color will be set in the ApplyState() method
                                         *
                                         * @param c_color color to be set
                                         * @see CColor
                                         *
                                         **/
                                        inline virtual void SetAllLedsColor(const CColor& c_color)
                                        {
                                            CHECK_IS_ACTUATOR_USED_HELPER(LEDS_ACTUATOR_XML_NAME, m_bIsUsingLeds, "SetAllLedsColor");
                                            m_bRefreshLeds = true;
                                            for (UInt8 i = 0; i < NUM_LEDS; i++) {
                                                m_tActuatedLedColors[i] = CColor(c_color.GetRed(), c_color.GetGreen(), c_color.GetBlue(), c_color.GetAlpha());
                                            }
                                        }

                                        /**
                                                 *
                                                 * @brief Sets the intensity for the specified LED
                                                 * notice that the actual intensity will be set in the ApplyState() method
                                                 *
                                                 * @param un_index index of the LED to be set
                                                 * @param un_intensity intensity of the color (0-255)
                                                 * @see CColor
                                                 * @see CCI_FootBotLedsActuator
                                                 *
                                                 **/
                                                inline virtual void SetSingleLedIntensity(UInt8 un_index, UInt8 un_intensity)
                                                {
                                                    CHECK_IS_ACTUATOR_USED_HELPER(LEDS_ACTUATOR_XML_NAME, m_bIsUsingLeds, "SetSingleLedIntensity");
                                                    m_bRefreshLeds = true;
                                                    m_tActuatedLedColors[un_index].SetAlpha(un_intensity);
                                                }

                                                /**
                                                 *
                                                 * @brief Sets the intensity for all the LEDs
                                                 * notice that the actual intensity will be set in the ApplyState() method
                                                 *
                                                 * @param un_intensity intensity of the color (0-255)
                                                 * @see CColor
                                                 * @see CCI_FootBotLedsActuator
                                                 *
                                                 **/
                                                inline virtual void SetAllLedsIntensity(UInt8 un_intensity)
                                                {
                                                    CHECK_IS_ACTUATOR_USED_HELPER(LEDS_ACTUATOR_XML_NAME, m_bIsUsingLeds, "SetAllLedsIntensity");
                                                    m_bRefreshLeds = true;
                                                    for (UInt32 i=0; i < NUM_LEDS; i++) {
                                                        m_tActuatedLedColors[i].SetAlpha(un_intensity);
                                                    }
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

                        /**
                                 *
                                 * @brief Returns the current target color for the specified LED
                                 * notice that the actual color could be different
                                 *
                                 * @param un_index index of the target LED
                                 * @return target led color
                                 * @see CColor
                                 * @see CCI_FootBotLedsActuator
                                 *
                                 **/
                                inline virtual CColor GetActuatedSingleLedColor(UInt8 un_index)
                                {
                                    CHECK_IS_ACTUATOR_USED_HELPER(LEDS_ACTUATOR_XML_NAME, m_bIsUsingLeds, "GetSingleActuatedLedColor");
                                    return m_tActuatedLedColors[un_index];
                                }

                                /**
                                 *
                                 * @brief Returns the current target color for all the LEDs
                                 * notice that the actual colors could be different
                                 *
                                 * @return target led color
                                 * @see CColor
                                 * @see CCI_FootBotLedsActuator
                                 *
                                 **/
                                inline virtual CCI_LEDsActuator::TSettings& GetActuatedAllLedColors()
                                {
                                    CHECK_IS_ACTUATOR_USED_HELPER(LEDS_ACTUATOR_XML_NAME, m_bIsUsingLeds, "GetAllActuatedLedColors");
                                    return m_tActuatedLedColors;
                                }

                                /** Number of LEDs */
                                        static const UInt8 NUM_LEDS;

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
                        CCI_LEDsActuator*            m_pcLedsActuator;

                        /** Actuators booleans */
                        bool m_bIsUsingRABActuator;
                        bool m_bIsUsingLeds;

                        /** Actuator refresh booleans */
                        bool m_bRefreshLeds;

                        /** Actuators commands variables */
                        CByteArray m_tRABPacketDataToSend;
                        CCI_LEDsActuator::TSettings   m_tActuatedLedColors;

                        /* XML actuator names */
                        static const std::string RAB_ACTUATOR_XML_NAME;
                        static const std::string LEDS_ACTUATOR_XML_NAME;
    };

}

#endif
