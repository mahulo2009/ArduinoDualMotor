#ifndef Robot_Factory_Dual_Motor_H
#define Robot_Factory_Dual_Motor_H

#include "RobotFactory.h"
#include "Wheel.h"
#include "L298NHardwareController.h"
#include "RosConfigArduinoDutyDualMotor.h"
#include "DifferentialWheeledRobot.h"

class RobotFactoryDualMotor : public RobotFactory
{

public:
      RobotFactoryDualMotor(RosConfigArduinoDutyDualMotor *robot_confing);

      virtual RobotBase *buildRobot();

      virtual WheelBase *buildWheel(int index);

protected:
      RosConfigArduinoDutyDualMotor *robot_confing;
};
#endif
