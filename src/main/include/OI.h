#ifndef OI_H
#define OI_H

#include "Constants.h"
#include <frc/XboxController.h>

class Driver_Interface {
    public:
        Driver_Interface();
        double GetThrottle();
        double GetStrafe();
        double GetRotation();

        bool GetResetHeading();
        bool GetBrake();
        bool GetBrakePressed();

        bool GetSetBrakeMode();
        bool GetDisableBrakeMode();

        bool LowSpeed();
        void SetRumble(double Rumble);

        bool DriverOverrideIgnore();
    private:
        frc::XboxController _driver_controller{UserInterface::Driver::DRIVER_CONTROLLER_PORT};
};

#endif
