#ifndef L298N_Hardware_Controller_H
#define L298N_Hardware_Controller_H

 #include "HardwareController.h"
 #include "Arduino.h"
 #include "Encoder.h"

//#define L298N_HARDWARE_CONTROLLER_DEBUG 1

class L298NHardwareController : public HardwareController {

    public:

        L298NHardwareController(double gain,
                                double offset,
                                int min_duty, 
                                int max_duty, 
                                int ticks_per_revolution,
                                double wheel_radious);

        void            attachPower(int pin);		                                         //attach the power pin selecting
		void            attachDirection(int pin1,int pin2);								//attach the direction pin
        void            attachEncoder( Encoder * encoder);								
        
        virtual void    velocity(double velocity);

        virtual double  getVelocity(double dt);
        virtual void    update(double dt);


    protected:
    
        virtual void    setupDirection(Wheel_Direction direction);    
        virtual void    power(double duty);

    private:

        double      gain_;			
        double      offset_;			
        int         min_duty_;				    //minimum duty
        int         max_duty_;				    //maximun duty
        int         pin_power_;				    //pin number for the motor power, pwm.
        int         pin_direction_1_;		        //pin number for the motor direction, H-bridge.	
        int         pin_direction_2_;		        //pin number for the motor direction, H-bridge.	
        int         duty_;						//duty currently sent to the motor.

        Encoder *   encoder_; //TODO MOVE THIS OUT OF HERE
        double      current_velocity_;
        long        previous_ticks_;	
        int         ticks_per_revolution_;
        double      wheel_radious_;
        double      factor_;

        double previous_command_time_;
};

#endif
