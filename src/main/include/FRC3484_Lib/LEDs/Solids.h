#ifndef SOLIDS_H
#define SOLIDS_H

#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include "FRC3484_Lib/utils/SC_Functions.h"
#include <frc/util/Color.h>

        frc::LEDPattern _solid_red{frc::LEDPattern::Solid(SC_GammaCorrection(frc::Color::kRed, 2.2))};
        frc::LEDPattern _solid_orange{frc::LEDPattern::Solid(SC_GammaCorrection(frc::Color::kOrange, 2.2))};
        frc::LEDPattern _solid_yellow{frc::LEDPattern::Solid(SC_GammaCorrection(frc::Color::kYellow, 2.2))};
        frc::LEDPattern _solid_green{frc::LEDPattern::Solid(SC_GammaCorrection(frc::Color::kGreen, 2.2))};
        frc::LEDPattern _solid_blue{frc::LEDPattern::Solid(SC_GammaCorrection(frc::Color::kBlue, 2.2))};
        frc::LEDPattern _solid_purple{frc::LEDPattern::Solid(SC_GammaCorrection(frc::Color::kPurple, 2.2))};
        frc::LEDPattern _solid_pink{frc::LEDPattern::Solid(SC_GammaCorrection(frc::Color::kPink, 2.2))};

        std::array<std::pair<double, frc::Color>, 3> first_level{std::pair{0.0, frc::Color::kBlack}, std::pair{0.4, frc::Color::kWhite}, std::pair{0.6, frc::Color::kBlack}};
        frc::LEDPattern _score_one_mask = frc::LEDPattern::Steps(first_level);
        frc::LEDPattern _one_segment{_solid_blue.Mask(_score_one_mask)};

        std::array<std::pair<double, frc::Color>, 5> second_level{std::pair{0.0, frc::Color::kBlack}, std::pair{0.2, frc::Color::kWhite}, std::pair{0.4, frc::Color::kBlack}, std::pair{0.6, frc::Color::kWhite}, std::pair{0.8, frc::Color::kBlack},};
        frc::LEDPattern _score_two_mask = frc::LEDPattern::Steps(second_level);
        frc::LEDPattern _two_segments{_solid_blue.Mask(_score_two_mask)};

        std::array<std::pair<double, frc::Color>, 5> third_level{std::pair{0.0, frc::Color::kWhite}, std::pair{0.2, frc::Color::kBlack}, std::pair{0.4, frc::Color::kWhite}, std::pair{0.6, frc::Color::kBlack}, std::pair{0.8, frc::Color::kWhite},};
        frc::LEDPattern _score_three_mask= frc::LEDPattern::Steps(third_level);
        frc::LEDPattern _three_segment{_solid_blue.Mask(_score_three_mask)};

        frc::LEDPattern _pulse{_solid_blue.Breathe(1_s)};

#endif