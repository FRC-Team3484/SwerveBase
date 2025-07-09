#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/geometry/Pose2d.h>


class PathfindingCommand
    : public frc2::CommandHelper<frc2::Command, PathfindingCommand>
{
public:
    PathfindingCommand(frc::Pose2d target_pose);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;
};
