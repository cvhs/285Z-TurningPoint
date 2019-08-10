namespace vex {
  class motor_special : public motor {
    private:
      uint32_t  _local_index;

    public:
      motor_special( int32_t index ) :  motor( index ), _local_index(index) {};  
      ~motor_special() {};

      // Allow overloading of base class methods
      using motor::spin;
      
      // This is similar, not quite the same, as new a method in the next (Nov ?) SDK release
      // voltage can be +/-12.0 volta or +/-12000 mV
      void spin( directionType dir, double voltage, voltageUnits units ) {
        // convert volts to mV is necessary
        int32_t voltage_mv = (units == voltageUnits::volt ? voltage * 1000.0 : voltage );

        // flip based on direction flag
        voltage_mv = (dir == directionType::fwd ? voltage_mv : -(voltage_mv) );

        if( voltage_mv == 0 ) {
          stop();
        } else {
          // send mV value to control motor open loop
          vexMotorVoltageSet( _local_index, voltage_mv );
        }
      }
  };
}

using namespace vex;

brain Brain;
controller Controller;

motor_special fw(PORT1);
