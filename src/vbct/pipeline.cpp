#include "pipeline.hpp"

#include <unordered_set>

#include "stage5.hpp"
#include "stage6.hpp"
#include "stage7.hpp"
#include "stage8.hpp"
#include "stage9.hpp"
#include "validate.hpp"
#include "vbs.hpp"

namespace vbct {

Contour apply_vbs(const Contour& contour, double x) {
    auto edges = contour.edges();
    if (edges.size() < 2) return contour;

    auto split_edges = vbs(edges, x);

    std::vector<Point2i> new_points;
    new_points.reserve(split_edges.size());
    new_points.push_back(split_edges.front().a);
    for (const auto& e : split_edges) new_points.push_back(e.b);
    new_points.pop_back();  // last point closes back to first; don't duplicate it

    std::unordered_set<Point2i> original(contour.points.begin(), contour.points.end());
    std::vector<bool> vbs_inserted;
    vbs_inserted.reserve(new_points.size());
    for (const auto& p : new_points) vbs_inserted.push_back(original.find(p) == original.end());

    Contour result;
    result.points = std::move(new_points);
    result.contour_id = contour.contour_id;
    result.vbs_inserted = std::move(vbs_inserted);
    return result;
}

VbctResult run_vbct(const std::vector<Contour>& contours, double x, int64_t dedup_epsilon) {
    std::vector<Contour> cleaned;
    cleaned.reserve(contours.size());
    for (const auto& c : contours) cleaned.push_back(collapse_near_duplicate_points(c, dedup_epsilon));  // stage 0.5 hygiene

    validate_contours(cleaned);  // stage 1

    std::vector<Contour> post_vbs;
    post_vbs.reserve(cleaned.size());
    for (const auto& c : cleaned) post_vbs.push_back(apply_vbs(c, x));  // stage 2

    Mesh mesh = triangulate(post_vbs);  // stages 3-4

    return {std::move(mesh), std::move(post_vbs)};
}

Stage10Result run_full_pipeline(
    const std::vector<Contour>& contours,
    double x,
    double threshold,
    double spacing,
    double phase_offset,
    std::optional<double> anchor_t0_frac,
    std::optional<double> other_wall_t0_frac,
    std::optional<bool> reverse_canonical_wall,
    bool crosshatch_enabled,
    std::optional<Point2> chain_anchor_point,
    std::optional<double> chain_left_near_t_frac,
    std::optional<double> chain_left_far_t_frac,
    std::optional<double> chain_right_near_t_frac,
    std::optional<double> chain_right_far_t_frac,
    const std::vector<std::vector<Point2>>& extra_clip_loops,
    bool transition_ring,
    const std::vector<Point2>& transition_chain_domain_identities,
    double transition_solid_fill_spacing) {
    VbctResult r14 = run_vbct(contours, x);
    Stage5Result r5 = run_stage5(r14.mesh, threshold);
    Stage6Result r6 = run_stage6(r5);
    Stage7Result r7 = run_stage7(r5, r6);
    Stage8Result r8 = run_stage8(r5, r6, r7);
    Stage9Result r9 = run_stage9(r5, r6, r7, r8, threshold);
    return run_stage10(
        r9, spacing, phase_offset, anchor_t0_frac, other_wall_t0_frac, reverse_canonical_wall, crosshatch_enabled, chain_anchor_point,
        chain_left_near_t_frac, chain_left_far_t_frac, chain_right_near_t_frac, chain_right_far_t_frac, extra_clip_loops,
        /*chain_swap_left_right=*/false, transition_ring, transition_chain_domain_identities, transition_solid_fill_spacing);
}

}  // namespace vbct
