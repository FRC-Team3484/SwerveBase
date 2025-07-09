#ifndef PATHFINDINGCONSTANTS_H
#define PATHFINDINGCONSTANTS_H

#include <units/length.h>
#include <units/angle.h>

namespace PathfindingConstants {
    constexpr int FINAL_ALIGN_EXIT = 1000000;
    constexpr units::inch_t FINAL_POSE_TOLERANCE = .3_in;
    constexpr units::degree_t FINAL_ANGLE_TOLERANCE = 1_deg;
}

#endif