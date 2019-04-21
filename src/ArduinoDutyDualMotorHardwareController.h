#ifndef Arduino_Duty_Dual_Motor_HardwareController_H
#define Arduino_Duty_Dual_Motor_HardwareController_H

 #include "HardwareController.h"
 #include "Arduino.h"

#define ARDUINIO_DUTY_DUAL_MOTOR_HARDWARE_CONTROLLER_DEBUG 1

class ArduinoDutyDualMotorHardwareController : public HardwareController {

    public:

        ArduinoDutyDualMotorHardwareController(double max_speed,int min_duty, int max_duty);

        void attachPower(int pin);		                                         //attach the power pin selecting
		void attachDirection(int pin1,int pin2);								//attach the direction pin

        
        virtual void velocity(double velocity);

    protected:
    
        virtual void setupDirection(Wheel_Direction direction);    
        virtual void power(double duty);

    private:

        double max_speed_;			    //maximun speed of the motor in radians per second.
        int min_duty_;				    //minimum duty
        int max_duty_;				    //maximun duty
        int pin_power_;				    //pin number for the motor power, pwm.
        int pin_direction_1_;		        //pin number for the motor direction, H-bridge.	
        int pin_direction_2_;		        //pin number for the motor direction, H-bridge.	
        int duty_;						//duty currently sent to the motor.
};

#endif
