#ifndef ROBOT_H
#define ROBOT_H

#include <units/time.h>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include "FRC3484_Lib/components/SC_Photon.h"
#include "FRC3484_Lib/components/SC_Pathfinding/SC_Pathfinding.h"

#include "subsystems/DrivetrainSubsystem.h"
#include "commands/teleop/TeleopDriveCommand.h"
#include "AutonGenerator.h"
#include "OI.h"
#include "Constants.h"
#include "Config.h"

class Robot : public frc::TimedRobot {
    public:
        Robot();
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
        // Controller Interfaces
        Driver_Interface* _oi_driver = new Driver_Interface();
        Operator_Interface* _oi_operator = new Operator_Interface();
        Testing_Interface* _oi_testing = new Testing_Interface();

        // Subsystems
        #ifdef VISION_ENABLED
        SC_Photon* _vision_ptr = new SC_Photon(VisionConstants::CAMERA_CONFIGS, VisionConstants::APRIL_TAG_LAYOUT, VisionConstants::POSE_STRATEGY);
        #else
        SC_Photon* _vision_ptr = nullptr;
        #endif

        #ifdef DRIVETRAIN_ENABLED
        DrivetrainSubsystem* _drivetrain = new DrivetrainSubsystem(DrivetrainConstants::SWERVE_CONFIGS_ARRAY, _vision_ptr , DrivetrainConstants::PIGEON_ID, DrivetrainConstants::DRIVETRAIN_CANBUS_NAME, _oi_operator);
        #else
        DrivetrainSubsystem* _drivetrain = nullptr;
        #endif

        #ifdef DRIVETRAIN_ENABLED
        SC_Pathfinding* _pathfinding = new SC_Pathfinding(_drivetrain, [this](){return _drivetrain->GetPose();}, VisionConstants::APRIL_TAG_LAYOUT);
        #else
        SC_Pathfinding* _pathfinding = nullptr;
        #endif

        #if defined (DRIVETRAIN_ENABLED)
        AutonGenerator* _auton_generator = new AutonGenerator(_drivetrain, _pathfinding);
        #else
        AutonGenerator* _auton_generator = nullptr;
        #endif

        // Command Groups
        frc2::CommandPtr _drive_state_commands = frc2::cmd::Parallel(
            #if defined DRIVETRAIN_ENABLED
            TeleopDriveCommand{_drivetrain, _oi_driver}.ToPtr(),
            #endif
            frc2::cmd::None()
        );

        // Autons
        std::optional<frc2::CommandPtr> _auton_command;

        // State Machines
        enum driver_states {
            drive,
        };
        driver_states _driver_robot_state = drive;

        // Elastic
        units::second_t _match_time = 0_s;
};

#endif 