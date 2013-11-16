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
 * @file common/control_interface/behavioral_toolkit/ci_robot_state.h
 *
 * @brief This file provides the control interface definition for the robot state
 * needed by the behavioral toolkit. In particular, this type represents
 * a generalization of the different robot state objects. Therefore, it
 * should encapsulate all the methods for accessing sensors and
 * actuators that are common to all the robots
 *
 * @author Eliseo Ferrante - <eferrant@ulb.ac.be>
 * @author Nithin Mathews  - <nmathews@ulb.ac.be>
 * @author Giovanni Pini   - <gpini@iridia.ulb.ac.be>
 */

#ifndef CI_ROBOT_STATE_H
#define CI_ROBOT_STATE_H

namespace argos {
    class CCI_RobotState;
}

#include "utility_classes/helper.h"
#include <argos3/core/control_interface/ci_controller.h>

namespace argos {

    class CCI_RobotState {

        public:

            /**
            * Constructor.
            * @param  pc_robot the robot object
            */
            CCI_RobotState(CCI_Controller* c_controller) :
                m_pcController(c_controller){
            }

            /**
            * Copy constructor.
            */
            CCI_RobotState(const CCI_RobotState& c_state) {
                *this = c_state;
            }

            /**
             * @brief Operator =.
             */
            CCI_RobotState& operator=(const CCI_RobotState& c_state) {

                if (this != &c_state) {
                    m_pcController = c_state.m_pcController;
                }

                return *this;
            }

            /**
             * @brief Destructor.
             */
            virtual ~CCI_RobotState() {
            }

            /**
             *
             * @brief Initializes sensors and actuators declared in the XMl configuration
             *
             **/
            virtual void Init();

            /**
             *
             * @brief Reads sensor data and sets the robot state accordingly
             *
             **/
            virtual void ReadState() {
            }

            /**
             *
             * @brief Applies the actuator commands using the information in the robot state
             *
             **/
            virtual void ApplyState();

            /**
             *
             * @brief Returns the ID of the robot
             * @return The id of the robot
             *
             **/
            virtual inline const std::string& GetRobotId() const {
                return m_pcController->GetId(); //TODO
            }

            /**
            *
            * @brief Returns a pointer to the CI robot object
            * @return Pointer to the CI robot object
            *
            **/
            virtual inline CCI_Controller& GetContoller() {
            	return *m_pcController;
            }

        protected:

            //Pointers to robot object
            CCI_Controller* m_pcController;

    };

}

#endif
