#include "AutonGenerator.h"

AutonGenerator::AutonGenerator(DrivetrainSubsystem* drivetrain, SC_Pathfinding* pathfinding)
    : _drivetrain{drivetrain}, _pathfinding{pathfinding} {
}

frc2::CommandPtr AutonGenerator::GetAutonomousCommand() {
    fmt::print("Testing offset and nearest pose math\n");

    std::vector<frc::Pose2d> poses = _pathfinding->GetAprilTagPoses({1, 2, 3});
    frc::Pose2d nearest_pose = _pathfinding->GetNearestPose(poses);

    fmt::print("Nearest pose: %f, %f\n", nearest_pose.Translation().X().to<double>(), nearest_pose.Translation().Y().to<double>());

    std::vector<frc::Pose2d> offset_poses = _pathfinding->ApplyOffsetsToPoses(poses, std::vector<frc::Pose2d>{frc::Pose2d{12_in, -12_in, 0_deg}});
    frc::Pose2d offset_nearest_pose = _pathfinding->GetNearestPose(offset_poses);

    fmt::print("Offset nearest pose: %f, %f\n", offset_nearest_pose.Translation().X().to<double>(), offset_nearest_pose.Translation().Y().to<double>());

    fmt::print("Done\n Testing pathfinding and alignment commands");

    return frc2::cmd::Sequence(
        // ID 1 is a red feeder station
        // TODO: Do we need a function to get a single april tag pose, instead of needing to index the result?
        _pathfinding->GetPathfindCommand(_pathfinding->GetAprilTagPoses({1})[0], 6_in, true),
        frc2::cmd::Parallel(
            _pathfinding->GetFinalAlignmentCommand(_pathfinding->GetAprilTagPoses({1})[0], true),
            _pathfinding->GetNearPoseCommand(_pathfinding->GetAprilTagPoses({1})[0], 1_in)
        )
    );
}
