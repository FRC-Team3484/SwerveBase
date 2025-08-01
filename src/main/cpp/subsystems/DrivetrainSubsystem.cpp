#include "subsystems/DrivetrainSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

//Path Planner Paths
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/DriverStation.h>

using namespace SC;

using namespace frc;
using namespace units;
using namespace pathplanner;

DrivetrainSubsystem::DrivetrainSubsystem(SC_SwerveConfigs swerve_config_array[4], SC_Photon* vision, int pigeon_id, std::string_view drivetrain_canbus_name, Operator_Interface* operator_interface)
        : _vision{vision}, _pigeon{pigeon_id, drivetrain_canbus_name}, _oi{operator_interface}
{
    if (NULL != swerve_config_array) {
        wpi::array<frc::Rotation2d, 4> headings{wpi::empty_array};
        for (int i = 0; i < 4; i++) {
            if (FL == i || BL == i) {
                _modules[i] = new SwerveModule(swerve_config_array[i], DrivetrainConstants::DrivePIDConstants::LeftPID, drivetrain_canbus_name);
            } else {
                _modules[i] = new SwerveModule(swerve_config_array[i], DrivetrainConstants::DrivePIDConstants::RightPID, drivetrain_canbus_name);
            }
            headings[i] = _modules[i]->GetPosition().angle;
        }


        kinematics.ResetHeadings(headings);

        RobotConfig config = RobotConfig::fromGUISettings();

        AutoBuilder::configure(
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetChassisSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ DriveRobotcentric(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        std::make_shared<PPHolonomicDriveController>(
            PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config,
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                auto alliance = DriverStation::GetAlliance();
                if (alliance) {
                    return alliance.value() == DriverStation::Alliance::kRed;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );

        SmartDashboard::PutBoolean("Drivetrain Diagnostics", false);
    }

    _pigeon.GetConfigurator().Apply(ctre::phoenix6::configs::Pigeon2Configuration{});
    _odometry = new SwerveDrivePoseEstimator<4>{
                                                    kinematics, 
                                                    GetHeading(), 
                                                    GetModulePositions(), 
                                                    frc::Pose2d{}, 
                                                    {0.1, 0.1, 0.1}, 
                                                    {1.0, 1.0, 1.0}};
    SetBrakeMode();

    frc::SmartDashboard::PutData("Field", &_field);
}

void DrivetrainSubsystem::Periodic() {
    if (_odometry == NULL) {
        fmt::print("Error: odometry accessed in Periodic before initialization");
    } else {
        _odometry->Update(GetHeading(), GetModulePositions());
        if (_vision != NULL)
            if (_vision != NULL && !_oi->GetIgnoreVision()){
            for (const SC::SC_CameraResults& results : _vision->GetCameraResults(GetPose())){
                wpi::array<double, 3> newStdDevs{results.Standard_Deviation(0), results.Standard_Deviation(1), results.Standard_Deviation(2)};
                _odometry->AddVisionMeasurement(results.Vision_Measurement, results.Timestamp, newStdDevs);
            }
        }
    }

    if (SmartDashboard::GetBoolean("Drivetrain Diagnostics", false)) {
        SmartDashboard::PutNumber("FL Encoder", _modules[FL]->GetPosition().angle.Degrees().value());
        SmartDashboard::PutNumber("FR Encoder", _modules[FR]->GetPosition().angle.Degrees().value());
        SmartDashboard::PutNumber("BL Encoder", _modules[BL]->GetPosition().angle.Degrees().value());
        SmartDashboard::PutNumber("BR Encoder", _modules[BR]->GetPosition().angle.Degrees().value());
        SmartDashboard::PutNumber("Gyro Heading", GetHeading().Degrees().value());
SmartDashboard::PutNumber("Odometry X", GetPose().X().value());
        SmartDashboard::PutNumber("Odometry Y", GetPose().Y().value());
    }

    _field.SetRobotPose(GetPose());
}

void DrivetrainSubsystem::Drive(meters_per_second_t x_speed, meters_per_second_t y_speed, radians_per_second_t rotation, bool open_loop) {
    DriveRobotcentric(ChassisSpeeds::FromFieldRelativeSpeeds(x_speed, y_speed, rotation, GetHeading()), open_loop);
}

void DrivetrainSubsystem::DriveRobotcentric(ChassisSpeeds speeds, bool open_loop) {
    auto states = kinematics.ToSwerveModuleStates(speeds);
    SetModuleStates(states, open_loop, true);
}

void DrivetrainSubsystem::SetModuleStates(wpi::array<SwerveModuleState, 4> desired_states, bool open_loop, bool optimize) {
    kinematics.DesaturateWheelSpeeds(&desired_states, SwerveModuleConstants::MAX_WHEEL_SPEED);
    for (int i = 0; i < 4; i++) {
        if (_modules[i] == NULL) {
            fmt::print("Error: Swerve Module accessed in Periodic before initialization");
        } else {
            _modules[i]->SetDesiredState(desired_states[i], open_loop, optimize);
        }
    }
}

Rotation2d DrivetrainSubsystem::GetHeading() {
    return _pigeon.GetRotation2d().RotateBy(_pigeon_offset);
}

void DrivetrainSubsystem::SetHeading(degree_t heading) {
    ResetOdometry(Pose2d(_odometry->GetEstimatedPosition().Translation(), Rotation2d(heading)));
    //fmt::print("Reset Head!!!!!!\n");
}

degrees_per_second_t DrivetrainSubsystem::GetTurnRate() {
    return degrees_per_second_t{-_pigeon.GetAngularVelocityZWorld().GetValue()};
}

Pose2d DrivetrainSubsystem::GetPose() {
    if (_odometry == NULL) {
        fmt::print("Error: odometry accesed in GetPose before initialization");
        return Pose2d{0_m, 0_m, 0_deg};
    } else {
        return _odometry->GetEstimatedPosition();
    }
}

void DrivetrainSubsystem::ResetOdometry(Pose2d pose) {
    if (_odometry == NULL) {
        fmt::print("Error: odometry accesed in ResetOdometry before initialization");

    }  else {
        _pigeon_offset = pose.Rotation().Degrees() - _pigeon.GetRotation2d().Degrees();

        _odometry->ResetPosition(GetHeading(), GetModulePositions(), pose);
    }
}

wpi::array<SwerveModulePosition, 4> DrivetrainSubsystem::GetModulePositions() {
    int checkNull = CheckNotNullModule();
    if (checkNull == 4) {
        return {_modules[FL]->GetPosition(), _modules[FR]->GetPosition(), _modules[BL]->GetPosition(), _modules[BR]->GetPosition()};
    } else {
        return {SwerveModulePosition(0_m, 0_deg), SwerveModulePosition(0_m, 0_deg), SwerveModulePosition(0_m, 0_deg), SwerveModulePosition(0_m, 0_deg)};
    }
}

ChassisSpeeds DrivetrainSubsystem::GetChassisSpeeds() {
    int checkNull = CheckNotNullModule();
    if (checkNull == 4) {
        return kinematics.ToChassisSpeeds({_modules[FL]->GetState(), _modules[FR]->GetState(), _modules[BL]->GetState(), _modules[BR]->GetState()});
    } else {
        return kinematics.ToChassisSpeeds({SwerveModuleState(0_mps, 0_deg), SwerveModuleState(0_mps, 0_deg), SwerveModuleState(0_mps, 0_deg), SwerveModuleState(0_mps, 0_deg)});
    }
}

void DrivetrainSubsystem::StopMotors() {
    if (_modules[FL] != NULL) {
        _modules[FL]->StopMotors();
    }
    if (_modules[FR] != NULL) {
        _modules[FR]->StopMotors();
    }
    if (_modules[BL] != NULL) {
        _modules[BL]->StopMotors();
    }
    if (_modules[BR] != NULL) {
        _modules[BR]->StopMotors();
    }
}

void DrivetrainSubsystem::ResetEncoders() {
    if (_modules[FL] != NULL) {
        _modules[FL]->ResetEncoder();
    }
    if (_modules[FR] != NULL) {
        _modules[FR]->ResetEncoder();
    }
    if (_modules[BL] != NULL) {
        _modules[BL]->ResetEncoder();
    }
    if (_modules[BR] != NULL) {
        _modules[BR]->ResetEncoder();
    }
    ResetOdometry(GetPose());
}

void DrivetrainSubsystem::SetCoastMode() {
    if (_modules[FL] != NULL) {
        _modules[FL]->SetCoastMode();
    }
    if (_modules[FR] != NULL) {
        _modules[FR]->SetCoastMode();
    }
    if (_modules[BL] != NULL) {
        _modules[BL]->SetCoastMode();
    }
    if (_modules[BR] != NULL) {
        _modules[BR]->SetCoastMode();
    }
}

void DrivetrainSubsystem::SetBrakeMode() {
    if (_modules[FL] != NULL) {
        _modules[FL]->SetBrakeMode();
    }
    if (_modules[FR] != NULL) {
        _modules[FR]->SetBrakeMode();
    }
    if (_modules[BL] != NULL) {
        _modules[BL]->SetBrakeMode();
    }
    if (_modules[BR] != NULL) {
        _modules[BR]->SetBrakeMode();
    }
}

int DrivetrainSubsystem::CheckNotNullModule() {
    int counter = 0;
    for (int i = 0; i < 4; i++) {
        if (_modules[i] != NULL) {
            counter++;
        }
    }
    return counter;
}

