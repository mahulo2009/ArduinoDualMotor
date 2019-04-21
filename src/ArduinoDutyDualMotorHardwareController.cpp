#include "ArduinoDutyDualMotorHardwareController.h"

ArduinoDutyDualMotorHardwareController::ArduinoDutyDualMotorHardwareController(double max_speed,int min_duty, int max_duty):
    max_speed_(max_speed), min_duty_(min_duty), max_duty_(max_duty)
{
}

void ArduinoDutyDualMotorHardwareController::attachPower(int pin)
{
    this->pin_power_ = pin;
    
    pinMode(pin_power_, OUTPUT);
    analogWrite(pin_power_,0); 
}

void ArduinoDutyDualMotorHardwareController::attachDirection(int pin1,int pin2)
{
	this->pin_direction_1_= pin1;
    this->pin_direction_2_= pin2;

	pinMode(pin_direction_1_, OUTPUT);
    pinMode(pin_direction_2_, OUTPUT);
}

void ArduinoDutyDualMotorHardwareController::setupDirection(Wheel_Direction direction) 
{
    switch(direction) {

        case FORWARD:

            digitalWrite(pin_direction_1_,LOW);
            digitalWrite(pin_direction_2_,HIGH);

            break;

        case BACKWARD:

            digitalWrite(pin_direction_1_,HIGH);
            digitalWrite(pin_direction_2_,LOW);

            break;
    }
}

void ArduinoDutyDualMotorHardwareController::velocity(double velocity)
{

    #ifdef ARDUINIO_DUTY_DUAL_MOTOR_HARDWARE_CONTROLLER_DEBUG
    Serial.print("ArduinoDutyDualMotorHardwareController::velocity:");
    Serial.print("\t");
    Serial.print(velocity);
    Serial.print("\n");
    #endif


    if (velocity < 0) {
		setupDirection(BACKWARD);	
	} else {
  		setupDirection(FORWARD);
	}

  	float duty = ( (max_duty_-min_duty_) * abs(velocity) ) / (max_speed_) + min_duty_;
    power(duty);
}

void ArduinoDutyDualMotorHardwareController::power(double duty)
{
    if (duty<=min_duty_)
        duty=0;
    if (duty>max_duty_)
        duty=max_duty_;

    this->duty_ = ceil(duty); 

    #ifdef ARDUINIO_DUTY_DUAL_MOTOR_HARDWARE_CONTROLLER_DEBUG
    Serial.print("ArduinoDutyDualMotorHardwareController::power:");
    Serial.print("\t");
    Serial.print(this->duty_);
    Serial.print("\n");
    #endif

    analogWrite(pin_power_,this->duty_); 
}
