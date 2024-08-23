#include "OI.h"
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace UserInterface::Driver;
using namespace UserInterface::Operator;
using namespace UserInterface::Testing;

// ----------
// Driver
// ----------

//     Motion Drive
Driver_Interface::Driver_Interface(){};
double Driver_Interface::GetThrottle() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(THROTTLE), DRIVER_JOYSTICK_DEADBAND);}
double Driver_Interface::GetStrafe() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(STRAFE), DRIVER_JOYSTICK_DEADBAND);}
double Driver_Interface::GetRotation() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(ROTATION), DRIVER_JOYSTICK_DEADBAND);}
//     Settings Drive
bool Driver_Interface::GetResetHeading() {return _driver_controller.GetRawButton(RESET_HEADING);}
bool Driver_Interface::GetBrake() {return _driver_controller.GetRawButton(BRAKE);}
bool Driver_Interface::GetBrakePressed() {return _driver_controller.GetRawButtonPressed(BRAKE);}

bool Driver_Interface::GetSetBrakeMode() {return _driver_controller.GetRawButtonPressed(ENABLE_BRAKE_MODE);}
bool Driver_Interface::GetDisableBrakeMode() {return _driver_controller.GetRawButtonPressed(DISABLE_BRAKE_MODE);}
bool Driver_Interface::LowSpeed() {return _driver_controller.GetRawAxis(LOW_SPEED) > 0.5;}

bool Driver_Interface::DriverOverrideIgnore() {return _driver_controller.GetRawButton(DRIVER_IGNORE);}
void Driver_Interface::SetRumble(double Rumble) {_driver_controller.SetRumble(frc::GenericHID::kBothRumble, Rumble);}