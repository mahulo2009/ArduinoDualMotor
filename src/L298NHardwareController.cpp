#include "L298NHardwareController.h"

L298NHardwareController::L298NHardwareController(double gain,int min_duty, int max_duty):
    gain_(gain), min_duty_(min_duty), max_duty_(max_duty),encoder_(0),previous_ticks_(0)
{
}

void L298NHardwareController::attachPower(int pin)
{
    this->pin_power_ = pin;
    
    pinMode(pin_power_, OUTPUT);
    analogWrite(pin_power_,0); 
}

void L298NHardwareController::attachDirection(int pin1,int pin2)
{
	this->pin_direction_1_= pin1;
    this->pin_direction_2_= pin2;

	pinMode(pin_direction_1_, OUTPUT);
    pinMode(pin_direction_2_, OUTPUT);
}

void L298NHardwareController::attachEncoder( Encoder * encoder)
{
    this->encoder_=encoder;
}							

void L298NHardwareController::setupDirection(Wheel_Direction direction) 
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

void L298NHardwareController::velocity(double velocity)
{

    #ifdef L298N_HARDWARE_CONTROLLER_DEBUG
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
  	float duty = gain_ * abs(velocity);
    power(duty);
}

void L298NHardwareController::power(double duty)
{
    if (duty<=min_duty_)
        duty=0;
    if (duty>max_duty_)
        duty=max_duty_;

    this->duty_ = ceil(duty); 

    #ifdef L298N_HARDWARE_CONTROLLER_DEBUG
    Serial.print("ArduinoDutyDualMotorHardwareController::power:");
    Serial.print(" duty\t");
    Serial.print(this->duty_);
    Serial.print("\n");
    #endif

    analogWrite(pin_power_,this->duty_); 
}


double  L298NHardwareController::getVelocity(double dt) 
{
    return current_velocity_;
}

void L298NHardwareController::update(double dt) 
{

    if (encoder_ != 0) 
    {
        long ticks_ = encoder_->read();
        current_velocity_ = ( ( (ticks_ - previous_ticks_) * 2 * PI ) / 1200 ) / dt; //TODO

        previous_ticks_ = ticks_;

        #ifdef L298N_HARDWARE_CONTROLLER_DEBUG
        Serial.print("ArduinoDutyDualMotorHardwareController::update:");
        Serial.print(" ticks\t");
        Serial.print(ticks_);
        Serial.print(" velocity\t");
        Serial.print(current_velocity_);
        Serial.print("\n");
        #endif
    }

}