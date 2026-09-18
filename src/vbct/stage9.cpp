#include "stage9.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>

namespace vbct {
namespace {

using PointList = std::vector<Point2>;

constexpr double kCloseEps = 1e-6;

// Raised in place of the Python reference's AssertionError, for the exact
// same reason: a Stage-6-skipped triangle wedge adjacent to a hub breaks
// the angular-adjacency assumption the splice relies on. Caught at each
// site the reference catches AssertionError -- see spec S:12.7's
// transactional-per-hub-commit safety net (kept, not expected to fire
// under the default single_point hub algorithm).
struct SpliceError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

bool points_close(const Point2& a, const Point2& b, double eps = kCloseEps) {
    return std::hypot(a.x - b.x, a.y - b.y) < eps;
}

double wall_length(const PointList& points) {
    double sum = 0;
    for (size_t i = 0; i + 1 < points.size(); ++i) sum += std::hypot(points[i + 1].x - points[i].x, points[i + 1].y - points[i].y);
    return sum;
}

double chain_length(const Chain& chain) { return wall_length(chain.points); }

// Merges hub identities connected by a short branch-to-branch bridge chain
// (VBCT-Hub-Fan-Fragmentation-Brief.md) -- Stage 5's own junction-hub
// construction (Correction 3) can, on real, densely-varying wall geometry,
// triangulate what is physically one small local junction as either a
// single combined hub or as several separate hubs linked by tiny stub
// chains, depending on incidental local point arrangement that differs
// from one layer to the next on an otherwise Z-invariant model. A
// branch-to-branch chain is never itself prune-eligible (see `candidates`
// below), so those stub links would otherwise always survive as if they
// were real structural bridges between genuinely separate junctions,
// reshuffling which arms get spliced together at each fragment
// independently even though the real, long wall arms never move. Reusing
// the same `threshold` the user already configures for "how short counts
// as noise" (rather than inventing a new distance) to decide which
// branch-to-branch chains are stub links: anything shorter is folded into
// its two endpoint hubs' shared identity below, and excluded from the
// incident list entirely -- it connects two names for the same physical
// junction, not a real arm to be pruned or kept.
struct HubUnionFind {
    std::map<NodeId, NodeId> parent;
    NodeId find(const NodeId& x) {
        auto it = parent.find(x);
        if (it == parent.end()) return parent[x] = x, x;
        if (it->second == x) return x;
        return it->second = find(it->second);
    }
    void unite(const NodeId& a, const NodeId& b) { parent[find(a)] = find(b); }
};

// nullopt if this wall doesn't actually touch hub_point at either end --
// see spec S:12.4's "non-converging tips" and the self-loop case named in
// the Python reference's own docstring.
std::optional<PointList> orient_from_hub(const PointList& points, const Point2& hub_point) {
    if (points_close(points.front(), hub_point)) return points;
    if (points_close(points.back(), hub_point)) return PointList(points.rbegin(), points.rend());
    return std::nullopt;
}

void append_contig(PointList& path, const PointList& seg) {
    if (!points_close(path.back(), seg.front())) throw SpliceError("non-contiguous splice");
    path.insert(path.end(), seg.begin() + 1, seg.end());
}

// Like append_contig, but tolerates seg not starting exactly where path
// currently ends -- bridges the gap with one straight connecting edge.
// Needed specifically for the entry-wall-to-exit-wall transition within a
// single pruned arm's visit -- see spec S:12.4.
void bridge_append(PointList& path, const PointList& seg) {
    if (!points_close(path.back(), seg.front())) path.push_back(seg.front());
    path.insert(path.end(), seg.begin() + 1, seg.end());
}

// Drops the leading hub point -- see spec S:12.3's splice-at-X-not-hub
// requirement.
PointList trim(const PointList& outward) { return PointList(outward.begin() + 1, outward.end()); }

struct Incident {
    int idx;
    std::string end;  // "start" | "end"
    double angle;
    std::optional<PointList> outward_left;
    std::optional<PointList> outward_right;
    bool pruned = false;
};

Incident build_incident(int chain_idx, const std::string& end, const Stage7Result& stage7,
                         const Domain& domain) {
    const Chain& chain = stage7.chains[chain_idx];
    Point2 hub_point = (end == "start") ? chain.points.front() : chain.points.back();
    double dx, dy;
    if (end == "start") {
        dx = chain.points[1].x - hub_point.x;
        dy = chain.points[1].y - hub_point.y;
    } else {
        dx = chain.points[chain.points.size() - 2].x - hub_point.x;
        dy = chain.points[chain.points.size() - 2].y - hub_point.y;
    }
    std::optional<PointList> left = orient_from_hub(domain.left->points, hub_point);
    std::optional<PointList> right = orient_from_hub(domain.right->points, hub_point);
    std::optional<PointList> outward_left = (end == "start") ? left : right;
    std::optional<PointList> outward_right = (end == "start") ? right : left;

    Incident inc;
    inc.idx = chain_idx;
    inc.end = end;
    inc.angle = std::atan2(dy, dx);
    inc.outward_left = std::move(outward_left);
    inc.outward_right = std::move(outward_right);
    return inc;
}

// The full boundary walk from `before`'s own far cap to `after`'s,
// visiting every pruned arm in the gap in turn -- spec S:12.4/S:12.5.
// Also returns each visited arm's tip's path-index, for callers that need
// to cut the walk into two separate walls.
std::pair<PointList, std::vector<int>> through_and_back(const Incident& before, const Incident& after,
                                                          const std::vector<Incident>& run) {
    PointList tmp = trim(*before.outward_left);
    PointList path(tmp.rbegin(), tmp.rend());
    std::vector<int> tip_indices;
    for (const auto& arm : run) {
        // bridge_append, not append_contig: a hub merged from several
        // separate physical junction points (HubUnionFind's own doc
        // comment) has arms whose own wall arrays don't literally touch at
        // this transition -- they're a few mm apart, not the same point --
        // so the strict exact-contiguity check would abort the whole hub's
        // splice (confirmed directly: "non-contiguous splice" on real
        // VBCT-Hub-Fan-Fragmentation-Brief.md geometry). bridge_append
        // already exists for exactly this: it inserts one straight
        // connecting edge when the gap is real, and is a no-op identical
        // to append_contig when the points already coincide, so this
        // changes nothing for every case that isn't a merged hub.
        bridge_append(path, trim(*arm.outward_right));
        tip_indices.push_back(static_cast<int>(path.size()) - 1);
        PointList exit_trimmed = trim(*arm.outward_left);
        bridge_append(path, PointList(exit_trimmed.rbegin(), exit_trimmed.rend()));
    }
    bridge_append(path, trim(*after.outward_right));
    return {path, tip_indices};
}

std::vector<double> cum_lengths(const PointList& path) {
    std::vector<double> cum{0.0};
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        cum.push_back(cum.back() + std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y));
    }
    return cum;
}

// K=1 and K>=3: `before`/`after` stay two separate walls, cut at whichever
// visited arm's tip lands closest to the walk's own midpoint by length --
// spec S:12.5.
std::pair<PointList, PointList> extend_between(const Incident& before, const Incident& after,
                                                 const std::vector<Incident>& run) {
    auto [path, tip_indices] = through_and_back(before, after, run);
    std::vector<double> cum = cum_lengths(path);
    double target = cum.back() / 2.0;
    int best = tip_indices[0];
    double best_val = std::abs(cum[best] - target);
    for (int t : tip_indices) {
        double v = std::abs(cum[t] - target);
        if (v < best_val) {
            best_val = v;
            best = t;
        }
    }
    PointList before_part(path.begin(), path.begin() + best + 1);
    PointList after_part(path.begin() + best, path.end());
    std::reverse(after_part.begin(), after_part.end());
    return {before_part, after_part};
}

// Closes `core` (a hub-wall's own material, both duplicate hub endpoints
// already stripped) back into a full ring by splicing each pruned arm in
// `run` onto it -- spec S:12.6.
PointList close_self_loop(const PointList& core, const std::vector<Incident>& run) {
    PointList path = core;
    for (const auto& arm : run) {
        std::optional<PointList> entry = arm.outward_right;
        std::optional<PointList> exit_wall = arm.outward_left;
        if (!entry.has_value() || !points_close(trim(*entry).front(), path.back())) {
            entry = arm.outward_left;
            exit_wall = arm.outward_right;
        }
        append_contig(path, trim(*entry));
        PointList exit_trimmed = trim(*exit_wall);
        bridge_append(path, PointList(exit_trimmed.rbegin(), exit_trimmed.rend()));
    }
    if (!points_close(path.back(), core.front())) throw SpliceError("self-loop did not close back onto itself");
    return path;
}

// Both survivors at this hub are the same branch/branch chain's own two
// ends (a self-loop bridge) -- spec S:12.6.
std::optional<std::pair<Domain, std::vector<int>>> close_self_loop_hub(const std::vector<Incident>& incidents,
                                                                         int pos_a, int pos_b, int n,
                                                                         const Domain& domain) {
    int idx = incidents[pos_a].idx;
    std::optional<Point2> hub_point;
    for (int pos : {pos_a, pos_b}) {
        for (const auto& side : {incidents[pos].outward_left, incidents[pos].outward_right}) {
            if (side.has_value()) hub_point = side->front();
        }
    }
    if (!hub_point.has_value()) return std::nullopt;

    auto touches_both_ends = [&](const PointList& points) {
        return points_close(points.front(), *hub_point) && points_close(points.back(), *hub_point);
    };

    PointList hub_wall, other_wall;
    std::string hub_slot;
    if (touches_both_ends(domain.left->points) && !touches_both_ends(domain.right->points)) {
        hub_wall = domain.left->points;
        other_wall = domain.right->points;
        hub_slot = "left";
    } else if (touches_both_ends(domain.right->points) && !touches_both_ends(domain.left->points)) {
        hub_wall = domain.right->points;
        other_wall = domain.left->points;
        hub_slot = "right";
    } else {
        return std::nullopt;  // unsupported configuration
    }

    std::vector<std::vector<Incident>> gaps;
    for (auto [start_pos, end_pos] : {std::pair{pos_a, pos_b}, std::pair{pos_b, pos_a}}) {
        std::vector<Incident> run;
        int p = (start_pos + 1) % n;
        while (p != end_pos) {
            run.push_back(incidents[p]);
            p = (p + 1) % n;
        }
        gaps.push_back(std::move(run));
    }
    int non_empty_count = 0, non_empty_idx = -1;
    for (int g = 0; g < static_cast<int>(gaps.size()); ++g) {
        if (!gaps[g].empty()) {
            ++non_empty_count;
            non_empty_idx = g;
        }
    }
    if (non_empty_count != 1) return std::nullopt;
    const std::vector<Incident>& run = gaps[non_empty_idx];

    PointList core(hub_wall.begin() + 1, hub_wall.end() - 1);
    PointList closed = close_self_loop(core, run);  // may throw SpliceError

    Domain result;
    result.kind = "ring";
    if (hub_slot == "left") {
        result.left = Wall{closed};
        result.right = Wall{other_wall};
    } else {
        result.left = Wall{other_wall};
        result.right = Wall{closed};
    }

    std::vector<int> consumed{idx};
    for (const auto& arm : run) consumed.push_back(arm.idx);
    return std::make_pair(result, consumed);
}

Point2 far_cap(const Incident& inc, const Domain& domain) {
    const Point2& hub_point = inc.outward_left->front();
    return points_close(*domain.cap_start, hub_point) ? *domain.cap_end : *domain.cap_start;
}

// K=2 only: a single wall spanning from `before`'s own far cap to
// `after`'s, threading through every pruned arm in the gap -- spec S:12.5.
PointList through_wall(const Incident& before, const Incident& after, const std::vector<Incident>& run) {
    PointList tmp = trim(*before.outward_left);
    PointList path(tmp.rbegin(), tmp.rend());
    for (const auto& arm : run) {
        // bridge_append, not append_contig -- see through_and_back's own
        // identical comment above; the same merged-hub gap can occur here.
        bridge_append(path, trim(*arm.outward_right));
        PointList exit_trimmed = trim(*arm.outward_left);
        bridge_append(path, PointList(exit_trimmed.rbegin(), exit_trimmed.rend()));
    }
    bridge_append(path, trim(*after.outward_right));
    return path;
}

// Splices `addition` (a single survivor's own K=1 wraparound extension,
// computed exactly as usual -- see that branch's own doc comment) onto
// `target`, an already-finalized chain domain from an earlier-processed hub
// that shared this same physical hub point before the extension moved
// `addition`'s hub-side cap away from it. Uses `bridge_append`, the same
// helper every other splice in this file already uses to tolerate two walls
// not exactly touching -- here the gap is bounded by `threshold` (the
// extension only ever moves a cap through one just-absorbed, sub-threshold
// stub), not an arbitrary distance.
void reopen_and_extend(Domain& target, const Domain& addition, const Point2& hub_point, double threshold) {
    bool target_at_start = points_close(*target.cap_start, hub_point);
    bool addition_at_start = points_close(*addition.cap_start, hub_point);

    // `target` and `addition` were each built independently (at two
    // different, possibly far-apart hubs), so there's no guarantee their
    // own left/right (inner/outer) labels agree -- confirmed directly on
    // real data (FPTF-45 (3), z=51.95): splicing left-to-left blindly
    // crossed the wall from inner to outer right at the seam. Decide which
    // pairing to use by comparing a point on each side well clear of the
    // seam. Neither the hub-adjacent point nor its immediate neighbour is
    // usable for this: left and right converge to the exact same
    // coordinate at a hub, and `addition`'s own hub-side end is the tip of
    // a just-absorbed stub, where both of its walls run along the same
    // sub-threshold contour feature (confirmed on the real case: one step
    // in, addition's left and right were the stub's two step-corner
    // vertices, both on the inner contour, and an exact vertex coincidence
    // with target's right made the wrong pairing win). Anything at least
    // `threshold` of wall length away from the seam is real wall by
    // construction (the extension only ever moved a cap through one
    // sub-threshold stub), so the two sides have genuinely separated there
    // and the correct pairing lands close together while the swapped one
    // is off by roughly the tube's own wall spacing.
    auto clear_point_of = [threshold](const std::optional<Wall>& w, bool at_start) -> std::optional<Point2> {
        if (!w.has_value() || w->points.size() < 2) return std::nullopt;
        const PointList& pts = w->points;
        size_t n = pts.size();
        double walked = 0.0;
        for (size_t i = 1; i < n; ++i) {
            size_t cur = at_start ? i : n - 1 - i;
            size_t prev = at_start ? i - 1 : n - i;
            walked += std::hypot(pts[cur].x - pts[prev].x, pts[cur].y - pts[prev].y);
            if (walked >= threshold) return pts[cur];
        }
        return at_start ? pts.back() : pts.front();  // wall shorter than threshold: use its far end
    };
    std::optional<Point2> t_left = clear_point_of(target.left, target_at_start);
    std::optional<Point2> t_right = clear_point_of(target.right, target_at_start);
    std::optional<Point2> a_left = clear_point_of(addition.left, addition_at_start);
    std::optional<Point2> a_right = clear_point_of(addition.right, addition_at_start);
    auto pt_dist = [](const Point2& a, const Point2& b) { return std::hypot(a.x - b.x, a.y - b.y); };
    bool swap_addition_sides = false;
    if (t_left.has_value() && t_right.has_value() && a_left.has_value() && a_right.has_value()) {
        double straight = pt_dist(*t_left, *a_left) + pt_dist(*t_right, *a_right);
        double swapped = pt_dist(*t_left, *a_right) + pt_dist(*t_right, *a_left);
        swap_addition_sides = swapped < straight;
    }
    const std::optional<Wall>& addition_left = swap_addition_sides ? addition.right : addition.left;
    const std::optional<Wall>& addition_right = swap_addition_sides ? addition.left : addition.right;

    auto splice_side = [&](std::optional<Wall>& target_wall, const std::optional<Wall>& addition_wall) {
        if (!target_wall.has_value() || !addition_wall.has_value()) return;
        PointList add_pts = addition_wall->points;
        if (!addition_at_start) std::reverse(add_pts.begin(), add_pts.end());
        // add_pts now runs hub-end -> far-end, regardless of addition's own original orientation.
        if (target_at_start) {
            PointList merged(add_pts.rbegin(), add_pts.rend());  // far-end -> hub-end
            bridge_append(merged, target_wall->points);          // -> target's own hub-end -> far-end
            target_wall = Wall{std::move(merged)};
        } else {
            PointList merged = target_wall->points;  // target's own far-end -> hub-end
            bridge_append(merged, add_pts);           // -> addition's hub-end -> far-end
            target_wall = Wall{std::move(merged)};
        }
    };
    splice_side(target.left, addition_left);
    splice_side(target.right, addition_right);

    const Point2& addition_far = addition_at_start ? *addition.cap_end : *addition.cap_start;
    if (target_at_start) target.cap_start = addition_far;
    else target.cap_end = addition_far;

    // The splice happened to close the loop -- promote to a proper Ring
    // rather than leaving a "chain" whose two caps coincide. Matches
    // close_self_loop_hub's own convention of not duplicating the shared
    // point across a closed wall's own front/back.
    if (target.cap_start.has_value() && target.cap_end.has_value() && points_close(*target.cap_start, *target.cap_end)) {
        target.kind = "ring";
        if (target.left.has_value() && target.left->points.size() > 1) target.left->points.pop_back();
        if (target.right.has_value() && target.right->points.size() > 1) target.right->points.pop_back();
        target.cap_start.reset();
        target.cap_end.reset();
    }
}

// ---- Hub-necklace merge (post-REV-2.2 fix) ---------------------------------
//
// Not in the spec as written. A hub whose *only* two incident chains are
// both un-prunable branch/branch bridges (n == 2, k == n -- nothing ever
// pruned there, since a branch/branch chain is never a candidate) is a
// pure Correction-3 fan artifact sitting on an otherwise plain closed
// loop: two Correction-3 hubs are enough to make a full circle produce
// this shape (each hub's own fan splits the loop it sits on into two
// arcs), and nothing before this pass ever collapses it back into a
// Ring. The main per-hub loop's `if (k == n) continue` skips such a hub
// entirely, and the K=2 "hub dissolves" merge (S:12.5) that *would*
// handle this never runs, because that branch is written for "K=2 as a
// result of pruning," not "K=2 because there were only ever 2 arms."
//
// Confirmed as the actual production mechanism (not a synthetic
// approximation), 2026-08-29: a real annulus, real STL geometry, real
// Cura build transform, run through the current pipeline (already
// carrying the S:11.6 self-loop-anchor fix) decomposed into several
// separate Chain domains around the ring, each with its own
// independently-computed left/right -- visible in production as a
// reversed Z-shift direction at each domain boundary, exactly spec
// S:11.4's "no cross-chain reconciliation is wanted" applied to a
// situation the spec never anticipated (chains meeting at a hub that
// should not have survived as a hub at all).
//
// This is a *cycle* problem, not a per-hub one: a necklace of N such
// hubs must be merged as a single unit, or a naive one-hub-at-a-time
// pass (even a second pass after the main loop) double-consumes a chain
// shared by two adjacent pass-through hubs processed independently.
// Detected and merged as whole connected cycles instead -- every node
// and edge in a qualifying cycle is visited exactly once, by
// construction, so there is no double-consumption to guard against.
//
// Left/right is reassigned per merged ring by distance from the cycle's
// own combined centroid (outer = farther, inner = nearer) rather than
// trying to reconcile each chain's own local left/right convention --
// sidesteps the reversed-orientation problem entirely instead of
// patching it, and is exactly as valid an assumption as the rest of
// this pipeline already makes about ring-shaped domains (S:13.3's
// winding-match step already assumes a "radial-ish" correspondence
// between a ring's two walls).
//
// Scope: only a *complete* cycle is merged. A chain of pass-through hubs
// that does not close on itself (e.g. it runs into a real, kept branch
// point) is left untouched -- conservative, matching this stage's
// existing "leave ambiguous configurations alone" precedent (S:12.6).
struct NecklaceEdge {
    int domain_idx;
    NodeId other_hub;
};

Point2 chain_centroid(const std::vector<Point2>& points) {
    double sx = 0, sy = 0;
    for (const auto& p : points) {
        sx += p.x;
        sy += p.y;
    }
    return {sx / points.size(), sy / points.size()};
}

double avg_dist(const std::vector<Point2>& points, const Point2& c) {
    double sum = 0;
    for (const auto& p : points) sum += std::hypot(p.x - c.x, p.y - c.y);
    return sum / points.size();
}

void merge_hub_necklaces(const Stage7Result& stage7, const std::vector<Domain>& chain_domains,
                          std::set<int>& removed_idx, const std::map<int, Domain>& extended_domain,
                          std::vector<Domain>& new_domains) {
    // "Pure pass-through" is a property of which *bridge* chains touch a
    // hub, not of the hub's total incident-chain count. Confirmed
    // necessary against a real production case (not a hypothetical): a
    // hub can carry a genuine dead_end/branch spur alongside its two
    // necklace-continuing bridges -- a few mm long, longer than the
    // configured pruning threshold, so it survives untouched and pushes
    // the hub's total degree to 3 without being a real branch. Gating on
    // total degree (REV 2.3's original check) missed this entirely; the
    // spur is simply irrelevant to whether the ring continues through
    // this hub, and is left as its own untouched Chain domain regardless
    // of what happens here.
    //
    // First pass: every still-untouched (never pruned, extended, or
    // otherwise consumed by the main per-hub loop) branch/branch chain
    // with two distinct hub endpoints (a genuine bridge -- a self-loop is
    // a different, already-handled case).
    std::vector<int> bridge_idx;
    for (int idx = 0; idx < static_cast<int>(stage7.chains.size()); ++idx) {
        const Chain& chain = stage7.chains[idx];
        if (chain.closed) continue;
        if (removed_idx.count(idx) || extended_domain.count(idx)) continue;
        if (chain.start_class != "branch" || chain.end_class != "branch") continue;
        if (!chain.start_node.has_value() || !chain.end_node.has_value()) continue;
        if (*chain.start_node == *chain.end_node) continue;  // self-loop; handled elsewhere
        bridge_idx.push_back(idx);
    }
    if (bridge_idx.empty()) return;

    // Tally how many such bridges touch each hub -- ignoring any other,
    // non-bridge chains (spurs) that might also meet there.
    std::map<NodeId, std::vector<int>> hub_to_bridges;
    for (int idx : bridge_idx) {
        const Chain& chain = stage7.chains[idx];
        hub_to_bridges[*chain.start_node].push_back(idx);
        hub_to_bridges[*chain.end_node].push_back(idx);
    }

    // A hub is a necklace candidate iff exactly 2 bridges touch it --
    // regardless of whatever else (a spur) is also incident there.
    std::set<NodeId> necklace_hubs;
    for (const auto& [hub, idxs] : hub_to_bridges) {
        if (idxs.size() == 2) necklace_hubs.insert(hub);
    }
    if (necklace_hubs.empty()) return;

    // Build the graph over just those hubs: an edge only counts if
    // *both* its ends are necklace candidates (one end leading to a
    // non-candidate -- e.g. a real >=3-bridge branch point -- doesn't
    // make this hub part of a clean cycle).
    std::map<NodeId, std::vector<NecklaceEdge>> graph;
    for (int idx : bridge_idx) {
        const Chain& chain = stage7.chains[idx];
        const NodeId& a = *chain.start_node;
        const NodeId& b = *chain.end_node;
        if (!necklace_hubs.count(a) || !necklace_hubs.count(b)) continue;
        graph[a].push_back({idx, b});
        graph[b].push_back({idx, a});
    }

    std::set<NodeId> visited_hubs;
    for (const auto& [start_hub, edges] : graph) {
        if (visited_hubs.count(start_hub)) continue;
        if (edges.size() != 2) continue;  // not a clean pass-through node in this graph

        // Walk the cycle starting at start_hub, picking either edge first.
        std::vector<int> cycle_domain_idx;
        std::vector<NodeId> cycle_hubs{start_hub};
        NodeId prev_hub = start_hub;
        NodeId cur_hub = start_hub;
        int incoming_domain_idx = -1;
        bool ok = true;
        do {
            auto it = graph.find(cur_hub);
            if (it == graph.end() || it->second.size() != 2) {
                ok = false;
                break;
            }
            const NecklaceEdge* next_edge = nullptr;
            for (const auto& e : it->second) {
                if (e.domain_idx != incoming_domain_idx) {
                    next_edge = &e;
                    break;
                }
            }
            if (!next_edge) {
                ok = false;  // e.g. a 2-cycle where both edges are the same domain idx somehow
                break;
            }
            cycle_domain_idx.push_back(next_edge->domain_idx);
            incoming_domain_idx = next_edge->domain_idx;
            prev_hub = cur_hub;
            cur_hub = next_edge->other_hub;
            if (cur_hub != start_hub) cycle_hubs.push_back(cur_hub);
        } while (cur_hub != start_hub && cycle_hubs.size() <= graph.size() + 1);
        (void)prev_hub;

        if (!ok || cur_hub != start_hub || cycle_domain_idx.size() < 2) continue;  // not a clean closed cycle

        // Classify each chain's two walls into outer/inner by distance
        // from the cycle's own combined centroid.
        std::vector<Point2> all_points;
        for (int idx : cycle_domain_idx) {
            const auto& d = chain_domains[idx];
            all_points.insert(all_points.end(), d.left->points.begin(), d.left->points.end());
            all_points.insert(all_points.end(), d.right->points.begin(), d.right->points.end());
        }
        Point2 centroid = chain_centroid(all_points);

        // Each hub's coordinate, once per cycle position -- needed both to
        // orient each spliced segment and to pick a deterministic splice
        // start below.
        std::vector<Point2> entry_hubs(cycle_domain_idx.size());
        for (size_t i = 0; i < cycle_domain_idx.size(); ++i) {
            const Chain& chain = stage7.chains[cycle_domain_idx[i]];
            entry_hubs[i] = (*chain.start_node == cycle_hubs[i]) ? chain.points.front() : chain.points.back();
        }

        // The merged wall's own points[0] must be a deterministic,
        // geometry-derived choice, not whichever hub this cycle walk (or
        // the graph's map iteration order) happened to start at --
        // otherwise it snaps between different hubs layer to layer for
        // near-identical geometry, breaking any downstream consumer that
        // needs a stable anchor (e.g. stage10's Z-layer phase-shift
        // reference-angle search). See
        // VBCT-Wall-Start-Point-Instability-Brief.md. Rule: the hub with
        // the lexicographically smallest (x, y) -- ties broken by the
        // hub NodeId itself, which can't tie on identical coordinates in
        // practice but keeps this fully deterministic regardless.
        size_t start_i = 0;
        for (size_t i = 1; i < entry_hubs.size(); ++i) {
            const Point2& best = entry_hubs[start_i];
            const Point2& cand = entry_hubs[i];
            bool better = (cand.x < best.x) || (cand.x == best.x && cand.y < best.y) ||
                          (cand.x == best.x && cand.y == best.y && cycle_hubs[i] < cycle_hubs[start_i]);
            if (better) start_i = i;
        }
        if (start_i != 0) {
            std::rotate(cycle_domain_idx.begin(), cycle_domain_idx.begin() + start_i, cycle_domain_idx.end());
            std::rotate(cycle_hubs.begin(), cycle_hubs.begin() + start_i, cycle_hubs.end());
            std::rotate(entry_hubs.begin(), entry_hubs.begin() + start_i, entry_hubs.end());
        }

        PointList outer_path, inner_path;
        bool splice_ok = true;
        try {
            for (size_t i = 0; i < cycle_domain_idx.size(); ++i) {
                const Domain& d = chain_domains[cycle_domain_idx[i]];
                const PointList *outer_wall, *inner_wall;
                if (avg_dist(d.left->points, centroid) >= avg_dist(d.right->points, centroid)) {
                    outer_wall = &d.left->points;
                    inner_wall = &d.right->points;
                } else {
                    outer_wall = &d.right->points;
                    inner_wall = &d.left->points;
                }
                // Both walls converge exactly at each hub vertex (S:11.3),
                // so either wall's own matching endpoint orients equally
                // well from this cycle position's (now canonical) entry hub.
                const Point2& entry_hub = entry_hubs[i];

                auto o_oriented = orient_from_hub(*outer_wall, entry_hub);
                auto i_oriented = orient_from_hub(*inner_wall, entry_hub);
                if (!o_oriented.has_value() || !i_oriented.has_value()) {
                    splice_ok = false;
                    break;
                }
                if (outer_path.empty()) {
                    outer_path = *o_oriented;
                    inner_path = *i_oriented;
                } else {
                    append_contig(outer_path, *o_oriented);
                    append_contig(inner_path, *i_oriented);
                }
            }
        } catch (const SpliceError&) {
            splice_ok = false;
        }
        if (!splice_ok) continue;
        if (!points_close(outer_path.back(), outer_path.front()) ||
            !points_close(inner_path.back(), inner_path.front())) {
            continue;  // didn't close cleanly; leave this cycle untouched
        }

        for (int idx : cycle_domain_idx) removed_idx.insert(idx);
        for (const auto& h : cycle_hubs) visited_hubs.insert(h);

        Domain ring;
        ring.kind = "ring";
        ring.left = Wall{outer_path};
        ring.right = Wall{inner_path};
        new_domains.push_back(std::move(ring));
    }
}

}  // namespace

Stage9Result run_stage9(const Stage5Result&, const Stage6Result&, const Stage7Result& stage7, const Stage8Result& stage8,
                         double threshold) {
    int n_chains = static_cast<int>(stage7.chains.size());
    std::vector<Domain> chain_domains(stage8.domains.begin(), stage8.domains.begin() + n_chains);
    std::vector<Domain> passthrough(stage8.domains.begin() + n_chains, stage8.domains.end());

    std::set<int> candidates;
    for (int idx = 0; idx < n_chains; ++idx) {
        const Chain& chain = stage7.chains[idx];
        if (chain.closed || chain.incomplete) continue;
        std::set<std::string> ends{*chain.start_class, *chain.end_class};
        if (ends != std::set<std::string>{"dead_end", "branch"}) continue;
        if (chain_length(chain) < threshold) candidates.insert(idx);
    }

    // Hub-fan fragmentation merge (see HubUnionFind's own doc comment): fold
    // together any two hub identities linked by a short branch-to-branch
    // chain, and mark that chain as absorbed rather than a real incident
    // arm at either end.
    HubUnionFind hub_union;
    std::set<int> absorbed_bridges;
    for (int idx = 0; idx < n_chains; ++idx) {
        const Chain& chain = stage7.chains[idx];
        if (chain.closed || chain.incomplete) continue;
        if (chain.start_class != "branch" || chain.end_class != "branch") continue;
        if (chain_length(chain) >= threshold) continue;
        hub_union.unite(*chain.start_node, *chain.end_node);
        absorbed_bridges.insert(idx);
    }

    // Insertion-order preserving map, matching Python dict insertion-order
    // semantics: a hub's processing order is not documented as canonical,
    // and a chain surviving at two different hubs has its
    // `extended_domain` entry silently overwritten by whichever hub is
    // processed second (see the loop below) -- so order fidelity here
    // isn't cosmetic, it can change which hub's extension a shared
    // survivor chain actually gets. Preserved exactly rather than risking
    // a silent behavioral divergence from the reference.
    std::vector<std::pair<NodeId, std::vector<std::pair<int, std::string>>>> hub_chain_ends;
    std::map<NodeId, size_t> hub_index;
    auto hub_entry = [&](const NodeId& hub) -> std::vector<std::pair<int, std::string>>& {
        auto it = hub_index.find(hub);
        if (it == hub_index.end()) {
            hub_index[hub] = hub_chain_ends.size();
            hub_chain_ends.push_back({hub, {}});
            return hub_chain_ends.back().second;
        }
        return hub_chain_ends[it->second].second;
    };
    for (int idx = 0; idx < n_chains; ++idx) {
        if (absorbed_bridges.count(idx)) continue;
        const Chain& chain = stage7.chains[idx];
        if (chain.closed) continue;
        if (chain.start_class == "branch") hub_entry(hub_union.find(*chain.start_node)).push_back({idx, "start"});
        if (chain.end_class == "branch") hub_entry(hub_union.find(*chain.end_node)).push_back({idx, "end"});
    }

    std::set<int> removed_idx;
    std::map<int, Domain> extended_domain;
    std::vector<Domain> new_domains;

    // Tracks which `new_domains` entry a chain idx became part of, for the
    // K=2 "both survivors dissolve" path only (self-loop/ring results have
    // no open end left to reopen onto). Lets a later-processed hub that
    // drops this chain as `already_consumed` (below) reopen and extend the
    // domain it was folded into, instead of leaving a gap where the two
    // domains used to meet -- see run_stage9's own doc comment at the drop
    // site for the real-data case this fixes.
    std::map<int, int> consumed_to_new_domain;

    // A branch-to-branch bridge chain shows up as one of *each* of its two
    // hubs' own incident lists (build_incident is per (chain, end) pair).
    // If an earlier-processed hub in this same loop already spliced that
    // chain into an extended domain (extended_domain[idx], written at the
    // end of every branch below), a later hub processing that same chain's
    // *other* end must build its own incident from that already-extended
    // domain, not the raw Stage 8 one -- otherwise the later hub's own
    // splice silently discards whatever the earlier hub already absorbed
    // into that chain (confirmed directly against a real production part:
    // a short spur correctly absorbed into a bridge chain at one hub was
    // being thrown away when a second hub, sharing that same bridge chain,
    // rebuilt it from the original un-extended wall).
    auto domain_for = [&](int idx) -> const Domain& {
        auto it = extended_domain.find(idx);
        return it != extended_domain.end() ? it->second : chain_domains[idx];
    };

    for (const auto& [hub, entries] : hub_chain_ends) {
        (void)hub;
        std::vector<Incident> incidents;
        for (const auto& [idx, end] : entries) incidents.push_back(build_incident(idx, end, stage7, domain_for(idx)));

        // A "keep" (non-prunable) chain that bridges two separate hubs shows up as one of
        // *each* hub's own incident entries (by design - build_incident is per (chain, end)
        // pair, and a bridge has a "branch" class at both ends). If an earlier-processed hub
        // in this same loop already spliced that chain into a new domain (removed_idx.insert
        // for its own idx, in every branch below), re-splicing it *again* here from this
        // hub's own, independently-built incidents would produce a second, overlapping domain
        // covering the same physical span through that shared chain - confirmed directly:
        // this is exactly the "two separate but very similar chain domains lying atop one
        // another" symptom reported on a real print, most visible once HubUnionFind's own
        // merge (above) makes it common for a single "keep" bridge to survive at two hubs
        // that both get spliced, though the underlying gap - `new_domains` has no
        // deduplication at all, unlike `extended_domain`'s own overwrite-on-reuse map - predates
        // that merge and could already fire without it.
        //
        // Rather than bailing this hub's *entire* incident list over one already-consumed
        // shared incident (the earlier, more conservative response -- see git history),
        // drop just that incident here and let this hub's own, otherwise-unrelated incidents
        // (a local prunable stub in particular) still get processed normally. This is safe
        // structurally: a chain is only ever incident at two hubs if *both* its ends are
        // "branch" (a bridge chain), so the only thing this filter can ever drop is a
        // previously-finalized bridge survivor -- never a genuine local prune candidate (those
        // are always {dead_end, branch}, incident at exactly one hub). Its geometry stays
        // exactly as the earlier hub already committed it; this hub never touches it again, so
        // the "two overlapping domains" hazard above can't recur. Confirmed directly against
        // real data (FPTF-45 (3), z=51.95): a 43mm-distant hub's own K=2 splice consumes a
        // shared 75mm bridge chain first, which used to bail this hub's own, completely
        // unrelated 1.6mm dead-end stub out of the 5mm prune threshold entirely; dropping only
        // the consumed bridge incident here lets the remaining single survivor absorb the stub
        // through the ordinary K=1 wraparound path below instead.
        //
        // An identical narrower fix was tried once before and reverted for regressing into
        // "artificially long chains with a void near one end" -- but that regression was
        // root-caused to the K=1 stale-cap bug, which is now fixed (see that branch's own doc
        // comment below); the failure mode that sank the earlier attempt no longer exists.
        std::vector<Incident> live_incidents;
        std::vector<std::pair<int, int>> reopen_targets;  // (dropped chain idx, new_domains index)
        for (auto& inc : incidents) {
            if (!removed_idx.count(inc.idx)) {
                live_incidents.push_back(std::move(inc));
                continue;
            }
            auto rt = consumed_to_new_domain.find(inc.idx);
            if (rt != consumed_to_new_domain.end()) reopen_targets.push_back({inc.idx, rt->second});
        }
        incidents = std::move(live_incidents);
        if (incidents.empty()) continue;  // nothing left at this hub to splice

        bool unreachable = false;
        for (const auto& inc : incidents) {
            if (!inc.outward_left.has_value() && !inc.outward_right.has_value()) unreachable = true;
        }
        if (unreachable) continue;  // not safe to splice; leave this hub's domains untouched

        for (auto& inc : incidents) inc.pruned = candidates.count(inc.idx) > 0;
        std::stable_sort(incidents.begin(), incidents.end(), [](const Incident& a, const Incident& b) { return a.angle < b.angle; });

        int n = static_cast<int>(incidents.size());
        int k = 0;
        for (const auto& inc : incidents)
            if (!inc.pruned) ++k;

        if (k == n) continue;  // nothing pruned at this hub

        if (k == 0) {
            for (const auto& inc : incidents) removed_idx.insert(inc.idx);
            std::vector<int> triangles;
            for (const auto& inc : incidents) {
                const auto& t = stage7.chains[inc.idx].triangles;
                triangles.insert(triangles.end(), t.begin(), t.end());
            }
            Domain d;
            d.kind = "glob";
            d.reason = "pruning_removed_all_arms";
            d.triangles = triangles;
            new_domains.push_back(std::move(d));
            continue;
        }

        std::vector<int> survivor_positions;
        for (int i = 0; i < n; ++i)
            if (!incidents[i].pruned) survivor_positions.push_back(i);

        if (survivor_positions.size() == 2 && incidents[survivor_positions[0]].idx == incidents[survivor_positions[1]].idx) {
            std::optional<std::pair<Domain, std::vector<int>>> ring;
            try {
                ring = close_self_loop_hub(incidents, survivor_positions[0], survivor_positions[1], n,
                                            domain_for(incidents[survivor_positions[0]].idx));
            } catch (const SpliceError&) {
                continue;
            }
            if (ring.has_value()) {
                for (int c : ring->second) removed_idx.insert(c);
                new_domains.push_back(ring->first);
            }
            continue;
        }

        if (survivor_positions.size() == 2) {
            int pos_a = survivor_positions[0], pos_b = survivor_positions[1];
            if (!incidents[pos_a].outward_left.has_value() || !incidents[pos_b].outward_right.has_value() ||
                !incidents[pos_b].outward_left.has_value() || !incidents[pos_a].outward_right.has_value()) {
                continue;  // can't safely splice this hub -- leave it untouched
            }
            std::vector<PointList> walls;
            std::set<int> local_removed;
            try {
                for (auto [start_pos, end_pos] : {std::pair{pos_a, pos_b}, std::pair{pos_b, pos_a}}) {
                    std::vector<Incident> run;
                    int p = (start_pos + 1) % n;
                    while (p != end_pos) {
                        run.push_back(incidents[p]);
                        local_removed.insert(incidents[p].idx);
                        p = (p + 1) % n;
                    }
                    walls.push_back(through_wall(incidents[start_pos], incidents[end_pos], run));
                }
            } catch (const SpliceError&) {
                continue;
            }
            int idx_a = incidents[pos_a].idx, idx_b = incidents[pos_b].idx;
            for (int c : local_removed) removed_idx.insert(c);
            removed_idx.insert(idx_a);
            removed_idx.insert(idx_b);

            PointList right_reversed(walls[1].rbegin(), walls[1].rend());
            Domain d;
            d.kind = "chain";
            d.left = Wall{walls[0]};
            d.right = Wall{right_reversed};
            d.cap_start = far_cap(incidents[pos_a], domain_for(idx_a));
            d.cap_end = far_cap(incidents[pos_b], domain_for(idx_b));
            new_domains.push_back(std::move(d));
            int new_idx = static_cast<int>(new_domains.size()) - 1;
            consumed_to_new_domain[idx_a] = new_idx;
            consumed_to_new_domain[idx_b] = new_idx;
            continue;
        }

        // K=1 (single survivor, one wrap-around gap) or K>=3 (per-gap, hub
        // stays a junction).
        std::map<int, std::map<std::string, PointList>> extensions;
        std::set<int> local_removed;
        bool aborted = false;
        try {
            for (size_t s = 0; s < survivor_positions.size(); ++s) {
                int pos_a = survivor_positions[s];
                int pos_b = survivor_positions[(s + 1) % survivor_positions.size()];
                std::vector<Incident> run;
                int p = (pos_a + 1) % n;
                while (p != pos_b) {
                    run.push_back(incidents[p]);
                    p = (p + 1) % n;
                }
                if (run.empty()) continue;
                if (!incidents[pos_a].outward_left.has_value() || !incidents[pos_b].outward_right.has_value()) {
                    continue;  // can't safely splice this gap -- leave those two survivors untouched
                }
                for (const auto& arm : run) local_removed.insert(arm.idx);
                auto [ext_before, ext_after] = extend_between(incidents[pos_a], incidents[pos_b], run);
                extensions[pos_a]["left"] = ext_before;
                extensions[pos_b]["right"] = ext_after;
            }
        } catch (const SpliceError&) {
            aborted = true;
        }
        if (aborted) continue;  // whole hub aborted; nothing committed yet

        for (int c : local_removed) removed_idx.insert(c);

        for (int pos : survivor_positions) {
            const Incident& inc = incidents[pos];
            auto ext_it = extensions.find(pos);
            std::optional<PointList> new_outward_left = inc.outward_left;
            std::optional<PointList> new_outward_right = inc.outward_right;
            if (ext_it != extensions.end()) {
                auto l = ext_it->second.find("left");
                if (l != ext_it->second.end()) new_outward_left = l->second;
                auto r = ext_it->second.find("right");
                if (r != ext_it->second.end()) new_outward_right = r->second;
            }
            std::optional<PointList> final_left = (inc.end == "start") ? new_outward_left : new_outward_right;
            std::optional<PointList> final_right = (inc.end == "start") ? new_outward_right : new_outward_left;

            const Domain& orig = domain_for(inc.idx);
            Domain d;
            d.kind = "chain";
            // Reference constructs Wall(points=None) if a side never
            // oriented and was never extended (untested in the 34-case
            // library, per spec S:12.6's own "not handled" scope note) --
            // storing nullopt here instead of crashing is strictly safer
            // and changes nothing for any case that actually exercises
            // this path.
            if (final_left.has_value()) d.left = Wall{*final_left};
            if (final_right.has_value()) d.right = Wall{*final_right};
            d.cap_start = orig.cap_start;
            d.cap_end = orig.cap_end;

            // K=1's own single-survivor wraparound (this branch's own doc comment above)
            // genuinely moves this incident's own hub-side end out to a brand new far tip --
            // wherever the wraparound's own cut point landed among the newly-absorbed pruned
            // arms -- it doesn't just add material behind the original hub-side cap. Leaving
            // that cap at its pre-extension value (as just assigned above) makes it silently
            // stale: it keeps labeling a location the domain's own wall no longer actually
            // reaches, and every consumer of a Chain domain's own cap_start/cap_end (Chain
            // end-position continuity tracking in particular) has no way to tell a stale cap
            // from a genuine one. Confirmed directly against a real production part: at a hub
            // with exactly one survivor, the resulting domain's own recorded cap sat at the old
            // hub location while its own wall geometry ran several mm further out through the
            // absorbed spurs - exactly the gap between "where the corrugation stops" and "where
            // the real wall actually ends" that produced a void near one end of the chain.
            //
            // Scoped to the single-survivor case specifically (`survivor_positions.size() == 1`,
            // so `ext_before`/`ext_after` both come from the one `extend_between` call above and
            // are guaranteed to converge at the same cut point - `final_left->back() ==
            // final_right->back()`) rather than generalized to every K>=3 gap: a K>=3 survivor
            // can be extended independently on its own left and right sides from two different
            // gaps, which have no reason to meet at a shared point, and this session's own
            // practice throughout has been to fix only the specific case real data confirms
            // rather than a broader generalization nothing has actually exercised yet.
            if (survivor_positions.size() == 1 && ext_it != extensions.end()) {
                const PointList* new_tip_wall = final_left.has_value()      ? &*final_left
                                                 : final_right.has_value()  ? &*final_right
                                                                             : nullptr;
                if (new_tip_wall != nullptr && !new_tip_wall->empty()) {
                    const Point2& new_tip = new_tip_wall->back();
                    if (inc.end == "start") {
                        d.cap_start = new_tip;
                    } else {
                        d.cap_end = new_tip;
                    }
                }
            }

            // This hub reduced to the simplest reopenable shape: one live
            // survivor, and the only other thing here was an already-
            // consumed bridge whose own domain we can still reach. Splice
            // this survivor's own extension directly onto that domain
            // instead of leaving both as separate domains with a gap where
            // this hub used to be -- see reopen_and_extend's own doc
            // comment and run_stage9's drop-site comment for the real-data
            // case (FPTF-45 (3), z=51.95) this fixes. Scoped narrowly to
            // exactly one survivor and exactly one reopen target, matching
            // this session's practice of fixing only the shape real data
            // confirms; every other shape keeps today's behavior.
            if (survivor_positions.size() == 1 && reopen_targets.size() == 1) {
                Point2 hub_point = inc.outward_left.has_value() ? inc.outward_left->front() : inc.outward_right->front();
                Domain& target = new_domains[reopen_targets[0].second];
                reopen_and_extend(target, d, hub_point, threshold);
                removed_idx.insert(inc.idx);
            } else {
                extended_domain[inc.idx] = std::move(d);
            }
        }
    }

    merge_hub_necklaces(stage7, chain_domains, removed_idx, extended_domain, new_domains);

    // Absorbed bridge chains (HubUnionFind's own doc comment above) were
    // deliberately excluded from every hub's own incident list -- not a
    // real arm to splice at either end -- so they were never visited by
    // the per-hub loop above and never added to `removed_idx` there. They
    // still need to be dropped here, or they'd sail through untouched as
    // their own tiny leftover domain instead of being folded into
    // whichever merged hub actually consumed their two endpoints.
    for (int idx : absorbed_bridges) removed_idx.insert(idx);

    std::vector<Domain> domains;
    for (int idx = 0; idx < n_chains; ++idx) {
        if (removed_idx.count(idx)) continue;
        auto it = extended_domain.find(idx);
        domains.push_back(it != extended_domain.end() ? it->second : chain_domains[idx]);
    }
    domains.insert(domains.end(), new_domains.begin(), new_domains.end());
    domains.insert(domains.end(), passthrough.begin(), passthrough.end());
    return {domains};
}

}  // namespace vbct
