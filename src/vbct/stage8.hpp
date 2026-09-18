// Stage 8: wall-pair extraction. Port of vbct/stage8.py -- see spec REV
// 2.1 S:11.
//
// Symbolic keys done properly (the plan's committed optimization). The
// Python reference's wall assembly (`_stitch`/`_dedupe_pieces`) keys a
// dict on *rounded float coordinates*, which the spec's own S:15.3
// profile names as ~67% of total pipeline time -- interpreter dispatch
// for the rounding, not geometric work. It doesn't need to: every piece
// endpoint this stage ever produces is a Stage-5 vertex the triangle
// already had (tip_a, b, or c -- see stage8.cpp's `triangle_pieces`),
// never an interpolated point invented on the fly. So this port keys
// pieces on the vertex *index* throughout -- an exact, zero-cost integer
// key -- and only materializes float coordinates once, at the very end,
// when building each Wall's point list for Stage 9/10 to consume.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "geometry.hpp"
#include "stage5.hpp"
#include "stage6.hpp"
#include "stage7.hpp"

namespace vbct {

struct Wall {
    std::vector<Point2> points;
};

struct Domain {
    std::string kind;  // "ring" | "chain" | "glob"
    std::optional<Wall> left;
    std::optional<Wall> right;
    std::optional<Point2> cap_start;  // "chain" only
    std::optional<Point2> cap_end;
    std::optional<std::string> reason;  // "glob" only: "incomplete_chain" | "skipped_triangles"
    std::vector<int> triangles;         // "glob" only
};

struct Stage8Result {
    std::vector<Domain> domains;
};

Stage8Result run_stage8(const Stage5Result& stage5, const Stage6Result& stage6, const Stage7Result& stage7);

}  // namespace vbct
