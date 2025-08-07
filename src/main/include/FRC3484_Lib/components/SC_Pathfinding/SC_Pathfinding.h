#ifndef SC_PATHFINDING_H
#define SC_PATHFINDING_H

#include <vector>

#include <units/length.h>

#include <frc/geometry/Pose2d.h>
#include <frc2/command/Commands.h>
#include <frc2/command/Subsystem.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include <pathplanner/lib/auto/AutoBuilder.h>

#include <FRC3484_Lib/components/SC_Pathfinding/PathfindingConstants.h>
#include <FRC3484_Lib/components/SC_Pathfinding/FinalAlignmentCommand.h>
#include <subsystems/DrivetrainSubsystem.h>

class SC_Pathfinding {
    public:
        /**
         * A library that handles april tags, pose math, and pathfinding commands
         * 
         * @param drivetrain_subsystem A pointer to the drivetrain subsystem
         * @param pose_supplier A lambda to the drivetrain function that returns the current pose of the robot
         * @param april_tag_layout The layout of the april tags
         */
        SC_Pathfinding(DrivetrainSubsystem *drivetrain_subsystem, std::function<frc::Pose2d()> pose_supplier, frc::AprilTagFieldLayout april_tag_layout);

        /**
         * Returns the poses of the april tags by ID
         * 
         * @param april_tag_ids The IDs of the april tags
         * 
         * @return The poses of the april tags as a vector
         */
        std::vector<frc::Pose2d> GetAprilTagPoses(std::vector<int> april_tag_ids);

        /**
         * Returns a pose with the given offset applied
         * 
         * @param pose The pose to apply the offset to
         * @param offset The offset to apply
         * 
         * @return The pose with the offset applied
         */
        frc::Pose2d ApplyOffsetToPose(frc::Pose2d pose, frc::Pose2d offset);

        /**
         * Returns a vector of poses with the given offsets applied
         * It returns the number of offsets equal to poses times offsets
         * Eg. if two poses and two offsets are provided, the returned vector will have 4 poses
         * 
         * @param poses The poses to apply the offsets to
         * @param offsets The offsets to apply
         * 
         * @return The poses with the offsets applied
         */
        std::vector<frc::Pose2d> ApplyOffsetsToPoses(std::vector<frc::Pose2d> poses, std::vector<frc::Pose2d> offsets);

        /** 
         * Returns the pose that is closest to the robot's current position
         * 
         * @param poses The poses to choose from
         * 
         * @return The closest pose
        */
        frc::Pose2d GetNearestPose(std::vector<frc::Pose2d> poses);

        /**
         * Returns a command that aligns the robot to the given pose
         * 
         * @param target The pose to align to
         * 
         * @return A command that aligns the robot to the given pose
         */
        frc2::CommandPtr GetFinalAlignmentCommand(frc::Pose2d target);

        /**
         * Returns a command that aligns the robot to the given pose
         * 
         * @param target The pose to align to
         * @param defer Whether to defer the command
         * 
         * @return A command that aligns the robot to the given pose
         */
        frc2::CommandPtr GetFinalAlignmentCommand(frc::Pose2d target, bool defer);

        /**
         * Returns a command that does nothing and waits until the robot is within a distance, then exits
         * Designed to be used in a ParallalCommandGroup with the GetFinalAlignmentCommand, 
         *  so once the robot is close to it's end position, the command group will exit
         * 
         * @param target The pose to wait for
         * @param distance The distance from the pose to wait for
         * 
         * @return A command that does nothing and waits until the robot is within a distance
         */
        frc2::CommandPtr GetNearPoseCommand(frc::Pose2d target, units::inch_t distance);

        /**
         * Returns a command that creates a path to go to the target pose
         * If a distance is provided, it should use a GetFinalAlignmentCommand 
         *  to align to the target once we're within that distance
         * 
         * @param target The pose to go to
         * @param distance The distance before aligning
         * @param defer Whether to defer the command
         * 
         * @return A command that creates a path to go to the target pose
         */
        frc2::CommandPtr GetPathfindCommand(frc::Pose2d target, units::inch_t distance, bool defer);

        /**
         * Returns a command that creates a path to go to the target pose
         * If a distance is provided, it should use a GetFinalAlignmentCommand 
         *  to align to the target once we're within that distance
         * 
         * @param target The pose to go to
         * @param distance The distance before aligning
         * 
         * @return A command that creates a path to go to the target pose
         */
        frc2::CommandPtr GetPathfindCommand(frc::Pose2d target, units::inch_t distance);

    private:
        DrivetrainSubsystem *_drivetrain_subsystem;
        std::function<frc::Pose2d()> _pose_supplier;
        frc::AprilTagFieldLayout _april_tag_layout;
};

#endif