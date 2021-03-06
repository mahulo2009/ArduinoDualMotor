#include "L298NHardwareController.h"


const float L298NHardwareController::frequency(93750);

L298NHardwareController::L298NHardwareController(double gain,
                                                 double offset,
                                                 int min_duty,
                                                 int max_duty,
                                                 int ticks_per_revolution) : gain_(gain),
                                                                         offset_(offset),
                                                                         ticks_per_revolution_(ticks_per_revolution),
                                                                         min_duty_(min_duty),
                                                                         max_duty_(max_duty),
                                                                         encoder_(0),
                                                                         previous_ticks_(0)
                                                                         
{
    factor_ = (2 * PI) / (ticks_per_revolution_);
}

void L298NHardwareController::attachPower(int pin)
{
    this->pin_power_ = pin;

    pinMode(pin_power_, OUTPUT);
    analogWriteFrequency(pin_power_, frequency);

    analogWrite(pin_power_, 0);
}

void L298NHardwareController::attachDirection(int pin1, int pin2)
{
    this->pin_direction_1_ = pin1;
    this->pin_direction_2_ = pin2;

    pinMode(pin_direction_1_, OUTPUT);
    pinMode(pin_direction_2_, OUTPUT);
}

void L298NHardwareController::attachEncoder(Encoder *encoder)
{
    this->encoder_ = encoder;
}

void L298NHardwareController::setupDirection(Wheel_Direction direction)
{
    switch (direction)
    {

    case FORWARD:

        digitalWrite(pin_direction_1_, LOW);
        digitalWrite(pin_direction_2_, HIGH);

        break;

    case BACKWARD:

        digitalWrite(pin_direction_1_, HIGH);
        digitalWrite(pin_direction_2_, LOW);

        break;
    }
}

void L298NHardwareController::velocity(double velocity)
{

#ifdef L298N_HARDWARE_CONTROLLER_DEBUG
    Serial.print("L298NHardwareController::velocity:");
    Serial.print("\t");
    Serial.print(velocity);
    Serial.print("\n");
#endif

    if (velocity < 0)
    {
        setupDirection(BACKWARD);
    }
    else
    {
        setupDirection(FORWARD);
    }
    float duty = gain_ * abs(velocity) + offset_;
    power(duty);
}

void L298NHardwareController::power(double duty)
{
    if (duty <= min_duty_)
        duty = 0;
    if (duty > max_duty_)
        duty = max_duty_;

    this->duty_ = ceil(duty);

#ifdef L298N_HARDWARE_CONTROLLER_DEBUG
    Serial.print("L298NHardwareController::power:");
    Serial.print(" duty\t");
    Serial.print(this->duty_);
    Serial.print("\n");
#endif

    analogWrite(pin_power_, this->duty_);
}

double L298NHardwareController::getVelocity()
{
    return current_velocity_;
}

void L298NHardwareController::update()
{
    if (encoder_ != 0)
    {

        double t = millis();

        long ticks_ = encoder_->read();
        long diff = ticks_ - previous_ticks_;
        current_velocity_ = factor_ * (diff) / ((t - previous_command_time_) / 1000.0);
        previous_ticks_ = ticks_;
        previous_command_time_ = t;

#ifdef L298N_HARDWARE_CONTROLLER_DEBUG
        Serial.print("L298NHardwareController::update:");
        Serial.print(" time\t");
        Serial.print(t - previous_command_time_);
        Serial.print(" ticks\t");
        Serial.print(diff);
        Serial.print(" velocity\t");
        Serial.print(current_velocity_ * 1000);
        Serial.print("\n");
#endif
    }
}