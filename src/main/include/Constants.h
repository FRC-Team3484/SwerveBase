
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/acceleration.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <photon/PhotonPoseEstimator.h>

#include <FRC3484_Lib/utils/SC_ControllerMaps.h>
#include <FRC3484_Lib/utils/SC_Datatypes.h>

namespace VisionConstants {
    const frc::AprilTagFieldLayout APRIL_TAG_LAYOUT = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeWelded);
    constexpr photon::PoseStrategy POSE_STRATEGY = photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR;

    const Eigen::Matrix<double, 3, 1> SINGLE_TAG_STDDEV{4, 4, 8};
    const Eigen::Matrix<double, 3, 1> MULTI_TAG_STDDEV{0.5, 0.5, 1};

    const std::vector<SC::SC_CameraConfig> CAMERA_CONFIGS = {
        SC::SC_CameraConfig{ // Front Left
            "Camera_1",
            frc::Transform3d{frc::Translation3d{10_in, 11.31_in, 8.75_in}, frc::Rotation3d{0_deg, -20.8_deg, -23.2_deg}},
            true
        },
        SC::SC_CameraConfig{ // Front Right
            "Camera_2",
            frc::Transform3d{frc::Translation3d{10_in, -11.31_in, 8.75_in}, frc::Rotation3d{0_deg, -20.8_deg, 23.2_deg}},
            true
        }
    };
}

namespace SwerveModuleConstants {
    constexpr units::feet_per_second_t MAX_WHEEL_SPEED = 8_fps;
    constexpr units::inch_t WHEEL_RADIUS = 2_in;
    constexpr double DRIVE_GEAR_RATIO = 36000.0/5880.0;
    constexpr double DRIVE_RATIO_SCALE = 1.0;
    namespace SteerPIDConstants {
        constexpr double Kp = 0.5;
        constexpr double Ki = 0.0;
        constexpr double Kd = 0.0;
        constexpr units::radians_per_second_t MAX_SPEED = 12_rad_per_s;
        constexpr units::radians_per_second_squared_t MAX_ACCELERATION = 100_rad_per_s_sq;
    }
}

namespace DrivetrainConstants {
    // For those with static, do not change into constants; it will break the linking
    // DO NOT REMOVE STATIC CALLS

    // Drive, steer, encoder, magnet offset
    static SC::SC_SwerveConfigs SWERVE_FRONT_LEFT{10,11,20, -90.527_deg}; //-92.505
    static SC::SC_SwerveConfigs SWERVE_FRONT_RIGHT{12,13,21, -58.096_deg}; //-60.205
    static SC::SC_SwerveConfigs SWERVE_BACK_LEFT{14,15,22, 160.225_deg}; //160.654
    static SC::SC_SwerveConfigs SWERVE_BACK_RIGHT{16,17,23, -55.898_deg}; //-55.283

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
    constexpr int PIGEON_ID = 22;
    constexpr std::string_view DRIVETRAIN_CANBUS_NAME = "Drivetrain CANivore";

    namespace DrivePIDConstants {
        // Check SC_Datatypes for the struct
        [[maybe_unused]] static SC::SC_SwervePID LeftPID{2, 0, 0, 2.0715 * 1_V / 1_mps, 0.17977 * 1_V / 1_mps_sq, 0.77607_V};
        [[maybe_unused]] static SC::SC_SwervePID RightPID{2, 0, 0, 2.0802 * 1_V / 1_mps, 0.30693 * 1_V / 1_mps_sq, 0.73235_V};
    }

    namespace BrakeConstants {
        constexpr auto DYNAMIC_BRAKE_SCALING = -0.02/1_in;
        constexpr units::second_t BRAKE_DELAY = .5_s;
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
        constexpr double DRIVER_RUMBLE_HIGH = 0.5;
        constexpr double DRIVER_RUMBLE_LOW = 0.2;
        constexpr double RUMBLE_STOP = 0;

        constexpr double DRIVER_JOYSTICK_DEADBAND = 0.02;
        // Motion
        constexpr int THROTTLE = XBOX_LS_Y;
        constexpr int STRAFE = XBOX_LS_X;
        constexpr int ROTATION = XBOX_RS_X;
        // Settings
        constexpr int RESET_HEADING = XBOX_BACK;
        constexpr int BRAKE = XBOX_X;
        constexpr int BRAKE_MODE = XBOX_RB;
        constexpr int DISABLE_BRAKE_MODE = XBOX_LB;
        constexpr int LOW_SPEED = XBOX_LT;

        // Override
        constexpr int DRIVER_OVERRIDE = XBOX_Y;
    }

    namespace Operator {
        constexpr int OPERATOR_CONTROLLER_PORT = 1;
        constexpr double OPERATOR_RUMBLE_HIGH = 0.5;
        constexpr double OPERATOR_RUMBLE_LOW = 0.2;
        constexpr double RUMBLE_STOP = 0; 

        // TODO: Set this button
        constexpr int IGNORE_VISION = 0;
    }

    namespace Testing {
        constexpr int TESTING_CONTROLLER_PORT = 2;
    }
}

#endif 