#ifndef PATHFINDINGCONSTANTS_H
#define PATHFINDINGCONSTANTS_H

#include <units/length.h>
#include <units/angle.h>

namespace PathfindingConstants {
    // Constants for PathfindingCommand
    constexpr units::meters_per_second_t MAX_VELOCITY = 3.0_mps;
    constexpr units::meters_per_second_squared_t MAX_ACCELERATION = 4.0_mps_sq;
    constexpr units::radians_per_second_t MAX_ANGULAR_VELOCITY = 540_deg_per_s;
    constexpr units::radians_per_second_squared_t MAX_ANGULAR_ACCELERATION = 720_deg_per_s_sq;

    // Constants for FinalAlignmentCommand
    constexpr int FINAL_ALIGN_EXIT = 1000000;
    constexpr units::inch_t FINAL_POSE_TOLERANCE = .3_in;
    constexpr units::degree_t FINAL_ANGLE_TOLERANCE = 1_deg;
}

#endif