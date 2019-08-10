#include "robot-config.h"
          
#define current_rpm fw.velocity(velocityUnits::rpm)


// Note: voltage passed to the motor ranges from -12 V to +12 V //

void drop_PID(int target) {
    double power = 0;
    double error = target - fw.velocity(velocityUnits::rpm);
    double last_error = 0;
    
    double kp = 0;
    double kd = 0;
    double ki = 0;
    
    while(abs(error) > 5){
        error = current_rpm;
        double calc_P = kp * error;
        double calc_D = ki * (error - last_error);
        double calc_I = 0;
        
        if(error < 20){
            calc_I = ki * (error + last_error);
        }
        
        power = calc_P + calc_D + calc_I;
        
        fw.spin(directionType::fwd, power, voltageUnits::volt);   //yea this really needs to be checked before we run so we don't trash smth
        
        this_thread::sleep_for(10);
    }
}

int main() {
    while(1){
        fw.spin(directionType::fwd, 170, velocityUnits::rpm);
        
        if(current_rpm < 150){                                    //this is just a guess, have to see how much rpm drops when you shoot the first ball
            drop_PID(100);                                        //drop to 100 rpm using secondary pid, need to figure out actual value, just guessed 100
            fw.spin(directionType::fwd, 100, velocityUnits::rpm);
            this_thread::sleep_for(1000);            
        }
    }
}
