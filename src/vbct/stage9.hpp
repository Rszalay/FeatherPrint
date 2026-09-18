// Stage 9: pruning. Port of vbct/stage9.py -- see spec REV 2.1 S:12.
// Removes skeleton chains too short to deserve their own corrugation
// span, folding their material into neighboring surviving chains' walls
// at the hub rather than discarding it.
#pragma once

#include "stage5.hpp"
#include "stage6.hpp"
#include "stage7.hpp"
#include "stage8.hpp"

namespace vbct {

struct Stage9Result {
    std::vector<Domain> domains;
};

Stage9Result run_stage9(const Stage5Result& stage5, const Stage6Result& stage6, const Stage7Result& stage7,
                         const Stage8Result& stage8, double threshold);

}  // namespace vbct
