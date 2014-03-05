//#include <argos2/common/utility/logging/argos_log.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/angles.h>

#define BEHAVIOR_NAME "[CBTFootbotOdometry]: "

#include "bt_footbot_odometry.h"

/****************************************/
/****************************************/

CBTFootbotOdometry::CBTFootbotOdometry(CCI_RobotData<CCI_FootBotState>* c_robot_data) :
CCI_Behavior<CCI_FootBotState> (c_robot_data, "bt_footbot_odometry"),
theta(0),
started(false),
delta_s(0),
x(0),
y(0){
}

/****************************************/
/****************************************/

CBTFootbotOdometry::~CBTFootbotOdometry() {

}

/****************************************/
/****************************************/

void CBTFootbotOdometry::Init(CCI_FootBotState& c_robot_state) {
	theta = CRadians(0);
	started = false;
	delta_s = 0;
	x = 0;
	y = 0;
}

/****************************************/
/****************************************/

void CBTFootbotOdometry::Step(CCI_FootBotState& c_robot_state) {
	CCI_DifferentialSteeringSensor::SReading reading = c_robot_state.GetSteeringReading();

	/*delta_s = (reading.CoveredDistanceLeftWheel+reading.CoveredDistanceRightWheel)/2.0;  // is negative in juan

		CRadians delta_theta = CRadians((reading.CoveredDistanceLeftWheel - reading.CoveredDistanceRightWheel)/reading.WheelAxisLength);
		theta += delta_theta;
		theta.SignedNormalize();

		Real delta_x = delta_s*Cos(theta);
		Real delta_y = delta_s*Sin(theta);
		x += delta_x;
		y += delta_y;*/

	CRadians delta_theta = CRadians((reading.CoveredDistanceLeftWheel - reading.CoveredDistanceRightWheel)/reading.WheelAxisLength);
	theta.SignedNormalize();
	delta_s = (reading.CoveredDistanceLeftWheel+reading.CoveredDistanceRightWheel)/2.0;
	Real delta_x = delta_s*Cos(theta+delta_theta/2);
	Real delta_y = delta_s*Sin(theta+delta_theta/2);
	//Real delta_x = delta_y;
	//Real delta_y = -delta_x;

	x += delta_x;
	y += delta_y;
	theta += delta_theta;



	//LOG << "RobotRotation: " << theta << "\n";
	//LOG << "RobotWrtFood: " << x << "," <<y << "\n";
}

void CBTFootbotOdometry::Destroy(CCI_FootBotState& c_robot_state) {

}

/****************************************/
/****************************************/

CVector2 CBTFootbotOdometry::GetReversedLocationVector(){

	CVector2 tmp = CVector2(-x,-y).Rotate(-theta);
	return CVector2(tmp.GetX(),-tmp.GetY());
	//return CVector2(x,y).Rotate(-theta);
	//return (-CVector2(x,y)).Rotate(-theta);
	//return CVector2(y,x).Rotate(theta);


	//LOG << "FoodPointer: " << CVector2(-x2,-y2) << "\n";
	//return CVector2(x2,y2);

	/*CRadians angleFoodToCurrentPos = ATan2(y,x);
	CRadians targetAngle = angleFoodToCurrentPos - theta*/ ;//+ CRadians(0).PI;

	//return CVector2(CVector2(x,y).Length(),targetAngle);

	/*CVector2 tmp = -CVector2(x,y);
	tmp.Rotate(angle);
	LOG << tmp.Angle() << "\n";
	return tmp;*/


}

void CBTFootbotOdometry::Reset(CCI_FootBotState& c_robot_state) {
	Init(c_robot_state);
}

CRadians CBTFootbotOdometry::GetAngle(){
	return CRadians(theta);
}

void CBTFootbotOdometry::Start(){
	started = true;
}

void CBTFootbotOdometry::Stop(){
	started = false;
}

Real CBTFootbotOdometry::GetDistance(){
	return delta_s;
}

/****************************************/
/****************************************/

