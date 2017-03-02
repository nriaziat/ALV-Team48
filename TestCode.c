#pragma config(Sensor, S1,     HTMC,           sensorI2CCustomFastSkipStates)
#pragma config(Motor,  motorA,          motor,         tmotorNXT, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "drivers/hitechnic-compass.h"

task main () {


  int _target = 0;

  int startHeading = HTMCsetTarget(HTMC);

  while(true) {
    _target = HTMCsetTarget(HTMC);
    nxtDisplayCenteredBigTextLine(2, "%d", _target - startHeading);
    motor[motorA] = .5 * (_target - startHeading);
  }
}
