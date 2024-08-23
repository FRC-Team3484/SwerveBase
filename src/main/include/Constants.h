#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "units/time.h"
#define EN_TESTING

#define DRIVE_ENABLED

#include <units/voltage.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/geometry/Pose2d.h>

// #include <ctre/Phoenix.h>

#include <FRC3484_Lib/utils/SC_ControllerMaps.h>
#include <FRC3484_Lib/utils/SC_Datatypes.h>

namespace SwerveConstants {
    namespace ControllerConstants {
        constexpr double DRIVER_RUMBLE_HIGH = 0.5;
        constexpr double DRIVER_RUMBLE_LOW = 0.2;

        constexpr double OPERATOR_RUMBLE_HIGH = 0.5;
        constexpr double OPERATOR_RUMBLE_LOW = 0.2;
        
        constexpr double RUMBLE_STOP = 0;
    }

    namespace DrivetrainConstants {
        // Swerve Module Configurations

        // For those with static, do not change into constants; it will break the linking
        // DO NOT REMOVE STATIC CALLS

        // Drive, steer, encoder, magnet offset
        static SC::SC_SwerveConfigs SWERVE_FRONT_LEFT{10,11,20, -90.527}; //-92.505
        static SC::SC_SwerveConfigs SWERVE_FRONT_RIGHT{12,13,21, -58.096}; //-60.205
        static SC::SC_SwerveConfigs SWERVE_BACK_LEFT{14,15,22, 160.225}; //160.654
        static SC::SC_SwerveConfigs SWERVE_BACK_RIGHT{16,17,23, -55.898}; //-55.283

        static SC::SC_SwerveConfigs SWERVE_CONFIGS_ARRAY[4] = {
            SWERVE_FRONT_LEFT,
            SWERVE_FRONT_RIGHT,
            SWERVE_BACK_LEFT,
            SWERVE_BACK_RIGHT
        };

        #define FL 0
        #define FR 1
        #define BL 2
        #define BR 3

        constexpr units::inch_t DRIVETRAIN_WIDTH = 24_in;
        constexpr units::inch_t DRIVETRAIN_LENGTH = 24_in;
        constexpr double DRIVE_GEAR_RATIO = 36000.0/5880.0;
        constexpr double STEER_GEAR_RATIO = 12.8;
        constexpr int DRIVE_TICKS_PER_REVOLUTION = 2048;
        constexpr int STEER_TICKS_PER_REVOLUTION = 2048;
        constexpr units::inch_t WHEEL_RADIUS = 2_in;

        constexpr units::feet_per_second_t MAX_WHEEL_SPEED = 8_fps;
        constexpr units::feet_per_second_squared_t MAX_WHEEL_ACCELERATION = 4_fps_sq;

        namespace DrivePIDConstants {
            static SC::SC_SwervePID LeftPID{2, 0, 0, 2.0715 * 1_V / 1_mps, 0.17977 * 1_V / 1_mps_sq, 0.77607_V};
            static SC::SC_SwervePID RightPID{2, 0, 0, 2.0802 * 1_V / 1_mps, 0.30693 * 1_V / 1_mps_sq, 0.73235_V};
        }

        namespace SteerPIDConstants {
            constexpr double Kp_Steer = 0.5;
            constexpr double Ki_Steer = 0.0;
            constexpr double Kd_Steer = 0.0;
            constexpr units::radians_per_second_t MAX_SPEED = 12_rad_per_s;
            constexpr units::radians_per_second_squared_t MAX_ACCELERATION = 100_rad_per_s_sq;
        }

        namespace JoystickScaling {
            constexpr double LOW_SCALE = 0.35;
        }
    }

    namespace AutonDriveConstants {
        // How fast the robot can move in autons
        constexpr units::feet_per_second_t MAX_LINEAR_SPEED = 8_fps;
        constexpr units::feet_per_second_squared_t MAX_LINEAR_ACCELERATION = 4_fps_sq;
        constexpr units::radians_per_second_t MAX_ROTATION_SPEED = 5.431_rad_per_s;
        constexpr units::radians_per_second_squared_t MAX_ROTATION_ACCELERATION = 2_rad_per_s_sq;

        constexpr units::inch_t POSITION_TOLERANCE = 2_in; // Drive to a position, when safe to quit
        constexpr units::degree_t ANGLE_TOLERANCE = 2_deg;
    }

    namespace PathDrivePIDConstants {
        constexpr double P = 5.0;
        constexpr double I = 0.0;
        constexpr double D = 0.0;
    }

    namespace PathRotationPIDConstants {
        constexpr double P = 2.0;
        constexpr double I = 0.0;
        constexpr double D = 0.0;
    }
}

namespace UserInterface {
    namespace Driver {
        constexpr int DRIVER_CONTROLLER_PORT = 0;
        constexpr double DRIVER_JOYSTICK_DEADBAND = 0.02;

        // Motion
        constexpr int THROTTLE = XBOX_LS_Y;
        constexpr int STRAFE = XBOX_LS_X;
        constexpr int ROTATION = XBOX_RS_X;

        // Settings
        constexpr int RESET_HEADING = XBOX_BACK;
        constexpr int BRAKE = XBOX_X;
        constexpr int ENABLE_BRAKE_MODE = XBOX_RB;
        constexpr int DISABLE_BRAKE_MODE = XBOX_LB;
        constexpr int LOW_SPEED = XBOX_LT;

        // Ignore
        constexpr int DRIVER_IGNORE = XBOX_Y;
    }

    namespace Operator {}
    namespace Testing {}
}

#endif
