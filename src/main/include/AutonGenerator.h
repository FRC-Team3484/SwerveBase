#ifndef AUTON_GENERATOR_H
#define AUTON_GENERATOR_H

#include <vector>
#include <units/length.h>

#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "FRC3484_Lib/components/SC_Pathfinding/SC_Pathfinding.h"

// Subsystems
#include "subsystems/DrivetrainSubsystem.h"

#include "Datatypes.h"

class AutonGenerator {
    public:
        /**
         * Generates the autonomous commands for the robot
         * Also displays the dropdowns in ShuffleBoard/Elastic
         * 
         * @param drivetrain The pointer to the drivetrain subsystem
         */
        AutonGenerator(
            DrivetrainSubsystem* drivetrain,
            SC_Pathfinding* pathfinding
        );

        /**
         * Gets the autonomous commands based on the selected params in the dropdowns
         * 
         * @return The autonomous command group
         */
        frc2::CommandPtr GetAutonomousCommand();
        
    private:
        DrivetrainSubsystem* _drivetrain;
        SC_Pathfinding* _pathfinding;
};

#endif
