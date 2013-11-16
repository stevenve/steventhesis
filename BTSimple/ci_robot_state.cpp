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
 * @file common/control_interface/behavioral_toolkit/ci_robot_state.cpp
 *
 * @brief This file provides the control interface implementation for the robot
 * state needed by the behavioral toolkit.
 * It is meant to contain the behavioral toolkit implementation that is shared
 * among all the robots (example: clock sensor)
 *
 * @author Giovanni Pini - <gpini@iridia.ulb.ac.be>
 */

#include "ci_robot_state.h"

namespace argos {


   /****************************************/
   /****************************************/

   void CCI_RobotState::Init(){
	  CCI_Sensor::TMap mapSensors = m_pcController->GetAllSensors();
	  CCI_Sensor::TMap::const_iterator itSensors;
   }

   /****************************************/
   /****************************************/

   void CCI_RobotState::ApplyState(){
   }

   /****************************************/
   /****************************************/

}

