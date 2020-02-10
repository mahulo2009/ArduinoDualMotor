#ifndef L298N_Hardware_Controller_H
#define L298N_Hardware_Controller_H

#include "Arduino.h"
#include "HardwareController.h"
#include "Encoder.h"

//#define L298N_HARDWARE_CONTROLLER_DEBUG 1

/**
 * The aim of this class is to control a L298 H-Bridge Motor Drive:
 * 
 * - It converts from velocity commands to duty cycle of the PWM.
 * - It controls maximun and minimun duty cycle for the drive. Sending a value
 * to low has the effect of not moving the wheel at all, sending a value
 * bigger than the maximun has not sense.
 * - It calculates the velocity rad/sec. 
 * 
 * In order to have a correct value for the velocity the update function 
 * must be called. This function calculats the ticks between to succesives 
 * calls to obtain the velocity.
 * 
 */
class L298NHardwareController : public HardwareController
{

public:

    static const float frequency;//Frecuency for the PWM
    
    /**
     * Class constructor
     * 
     * Both gain and offset are used to transform from target velocity to duty:
     *  duty = vel * gain + offset
     * These values needs to be calibrated.
     * 
     * @param gain gain to transform from velocity to duty.
     * @param offset offset to transform from velocity to duty.
     * @param min_duty in case final duty minor than this, a 0 duty is applied
     * @param max_duty in case final duty bigger than this, this value is applied.
     * @param ticks_per_revolution number of ticks for a complete revolution
     */
    L298NHardwareController(double gain,
                            double offset,
                            int min_duty,
                            int max_duty,
                            int ticks_per_revolution);

    /**
     * Indicates the micro-controller pin to be used for the power (PWM)
     * 
     * @param pin pin number
     */
    void attachPower(int pin);
    /**
    * Indicates the micro-controller pins to be used for controlling direction.
    * 
    * Seding HIGH, LOW to these pins the motor moves fordware, sending LOW, HIGH the motor 
    * moves backware.
    * 
    * @param pin1 pin1 number
    * @param pin2 pin2 number
     */
    void attachDirection(int pin1, int pin2);
    /**
     * Attach the Encoder class to be used for reading ticks.
     * 
     * @param pointer to encoder class
     */
    void attachEncoder(Encoder *encoder);
    /**
     * Inline function with the last duty applied.
    */
    virtual int duty()
    {
        return duty_;
    }
    /**
     * Set the target velocity
     * 
     * @param target angular velocity in rad/sec
     */
    virtual void velocity(double velocity);
    /**
     * Return the last calculated angular velocity in rad/sec
     */
    virtual double getVelocity();
    /**
    *  Function to calculate the velocity, the velocity is calculated based on the increments ticks 
    * between two sucessives calls to this function
    */
    virtual void update();

protected:
    /**
     *  Set the Wheel movement direction. Sending the control values.
     */
    virtual void setupDirection(Wheel_Direction direction);
    /*
    *   Set the Wheel velocity. Sending the duty to the pwm pin.
    */
    virtual void power(double duty);

private:
    double gain_;              //gain to transform from velocity to duty
    double offset_;            //offset to transform from velocity to duty
    int ticks_per_revolution_; //number of ticks for a complete revolution
    int min_duty_;             //minimum duty
    int max_duty_;             //maximun duty
    int pin_power_;            //pin number for the motor power, pwm.
    int pin_direction_1_;      //pin number for the motor direction, H-bridge.
    int pin_direction_2_;      //pin number for the motor direction, H-bridge.

    double factor_; //factor to convert from tick per revolution to radians per second.

    Encoder *encoder_;             //point to encder class to read encoder ticks
    int duty_;                     //duty sent to the motor.
    double current_velocity_;      //last velocity calculated.
    long previous_ticks_;          //ticks read in a previous call to update.
    double previous_command_time_; //time read in a previous call to update.
};

#endif
