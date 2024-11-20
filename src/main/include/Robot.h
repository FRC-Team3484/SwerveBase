// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef ROBOT_H
#define ROBOT_H

#include "OI.h"
#include "Constants.h"
#include "AutonGenerator.h"

#include "subsystems/DrivetrainSubsystem.h"
#include "commands/teleop/TeleopDriveCommand.h"

#include <frc/TimedRobot.h>
#include "frc2/command/Commands.h"
#include <frc2/command/CommandPtr.h>
#include <frc/PowerDistribution.h>

class Robot : public frc::TimedRobot {
    public:
        void RobotInit() override;
        void RobotPeriodic() override;
        void DisabledInit() override;
        void DisabledPeriodic() override;
        void DisabledExit() override;
        void AutonomousInit() override;
        void AutonomousPeriodic() override;
        void AutonomousExit() override;
        void TeleopInit() override;
        void TeleopPeriodic() override;
        void TeleopExit() override;
        void TestInit() override;
        void TestPeriodic() override;
        void TestExit() override;

    private:
        // State machine
        enum State {drive};
        State _robot_state = drive;

        // Power Stuff
        frc::PowerDistribution _pdp{1, frc::PowerDistribution::ModuleType::kRev};

        // Interface OI
        Driver_Interface _oi_driver{};
        Operator_Interface _oi_operator{};

        // Subsystems
        #ifdef DRIVE_ENABLED
        DrivetrainSubsystem _drivetrain{SwerveConstants::DrivetrainConstants::SWERVE_CONFIGS_ARRAY};
        #endif

        AutonGenerator _auton_generator{&_drivetrain};

        // Command Groups
        frc2::CommandPtr _drive_state_commands = frc2::cmd::Parallel(
            #ifdef DRIVE_ENABLED
            TeleopDriveCommand{&_drivetrain, &_oi_driver}.ToPtr(),
            #endif
            frc2::cmd::None()
        );

        // Variables
        std::optional<frc2::CommandPtr> _auton_command;
};

#endif