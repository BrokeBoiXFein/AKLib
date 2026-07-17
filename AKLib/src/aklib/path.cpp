/**
 * \file path.cpp
 * Bezier sampling, curvature-aware speed planning, and the pure-pursuit
 * lookahead search.
 */

#include "aklib/path.hpp"

#include <algorithm>
#include <cmath>

namespace aklib {

namespace {

/** Point on a cubic Bezier at parameter t in [0, 1]. */
Point bezierPoint(const Point& p0, const Point& c0, const Point& c1,
                  const Point& p1, double t) {
    const double u = 1.0 - t;
    const double b0 = u * u * u;
    const double b1 = 3 * u * u * t;
    const double b2 = 3 * u * t * t;
    const double b3 = t * t * t;
    return {b0 * p0.x + b1 * c0.x + b2 * c1.x + b3 * p1.x,
            b0 * p0.y + b1 * c0.y + b2 * c1.y + b3 * p1.y};
}

/** Menger curvature of three consecutive points: 1/R of the circle through
 *  them. 0 for collinear points. */
double curvature(const Point& a, const Point& b, const Point& c) {
    const double area2 = std::fabs((b.x - a.x) * (c.y - a.y) -
                                   (b.y - a.y) * (c.x - a.x));
    const double d1 = a.distTo(b), d2 = b.distTo(c), d3 = a.distTo(c);
    const double denom = d1 * d2 * d3;
    if (denom < 1e-9) return 0;
    return 2.0 * area2 / denom;
}

} // namespace

Path Path::bezier(const std::vector<Point>& controls,
                  const PathOptions& options) {
    Path path;
    // Need 4 points for the first segment, then +3 per extra segment.
    if (controls.size() < 4 || (controls.size() - 4) % 3 != 0) return path;

    // Oversample each segment finely, then resample to even spacing.
    std::vector<Point> raw;
    const std::size_t numSegments = 1 + (controls.size() - 4) / 3;
    for (std::size_t s = 0; s < numSegments; s++) {
        const std::size_t base = s * 3;
        const Point& p0 = controls[base];
        const Point& c0 = controls[base + 1];
        const Point& c1 = controls[base + 2];
        const Point& p1 = controls[base + 3];
        constexpr int STEPS = 200;
        for (int i = (s == 0 ? 0 : 1); i <= STEPS; i++) {
            raw.push_back(bezierPoint(p0, c0, c1, p1, double(i) / STEPS));
        }
    }

    // Resample at even arc-length spacing.
    path.pts_.push_back({raw.front(), options.maxSpeed, 0});
    double carried = 0;
    for (std::size_t i = 1; i < raw.size(); i++) {
        double segLen = raw[i - 1].distTo(raw[i]);
        while (carried + segLen >= options.spacing) {
            const double need = options.spacing - carried;
            const double t = need / segLen;
            const Point p = {raw[i - 1].x + (raw[i].x - raw[i - 1].x) * t,
                             raw[i - 1].y + (raw[i].y - raw[i - 1].y) * t};
            path.pts_.push_back({p, options.maxSpeed, 0});
            // continue within this raw segment
            segLen -= need;
            raw[i - 1] = p;
            carried = 0;
        }
        carried += segLen;
    }
    if (path.pts_.back().p.distTo(raw.back()) > options.spacing * 0.25) {
        path.pts_.push_back({raw.back(), options.maxSpeed, 0});
    }

    path.finalize_(options);
    return path;
}

Path Path::line(Point a, Point b, const PathOptions& options) {
    Path path;
    const double len = a.distTo(b);
    const int n = std::max(2, int(len / options.spacing) + 1);
    for (int i = 0; i < n; i++) {
        const double t = double(i) / (n - 1);
        path.pts_.push_back({{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t},
                             options.maxSpeed, 0});
    }
    path.finalize_(options);
    return path;
}

void Path::finalize_(const PathOptions& options) {
    if (pts_.size() < 2) return;

    // ---- cumulative arc length ------------------------------------------
    for (std::size_t i = 1; i < pts_.size(); i++) {
        pts_[i].distance = pts_[i - 1].distance +
                           pts_[i - 1].p.distTo(pts_[i].p);
    }

    // ---- forward pass: slow down for curvature ---------------------------
    for (std::size_t i = 1; i + 1 < pts_.size(); i++) {
        const double k = curvature(pts_[i - 1].p, pts_[i].p, pts_[i + 1].p);
        const double curveSpeed = k > 1e-6 ? options.turnK / k : options.maxSpeed;
        pts_[i].speed = clamp(std::min(options.maxSpeed, curveSpeed), 0.05,
                              options.maxSpeed);
    }
    pts_.back().speed = std::max(options.endSpeed, 0.05);

    // ---- backward pass: respect deceleration into slow points ------------
    // v[i] <= sqrt(v[i+1]^2 + 2 * decel * gap): can't be going faster here
    // than we can shed by the time we reach the slower point ahead.
    for (std::size_t i = pts_.size() - 1; i-- > 0;) {
        const double gap = pts_[i + 1].distance - pts_[i].distance;
        const double reachable = std::sqrt(
            pts_[i + 1].speed * pts_[i + 1].speed + 2 * options.decel * gap);
        pts_[i].speed = std::min(pts_[i].speed, reachable);
    }
}

std::size_t Path::closestIndex(const Point& p, std::size_t fromIndex) const {
    std::size_t best = fromIndex;
    double bestDist = 1e18;
    for (std::size_t i = fromIndex; i < pts_.size(); i++) {
        const double d = pts_[i].p.distTo(p);
        if (d < bestDist) { bestDist = d; best = i; }
    }
    return best;
}

Point Path::lookaheadPoint(const Point& robot, double lookahead,
                           std::size_t& startIndex) const {
    if (pts_.empty()) return robot;

    // Advance the progress index to the nearest point (never backward).
    startIndex = closestIndex(robot, startIndex);

    // Near the end: just aim at the final point.
    const double remaining = length() - pts_[startIndex].distance;
    if (remaining <= lookahead) return pts_.back().p;

    // Walk forward for the segment that crosses the lookahead circle, then
    // interpolate the exact crossing on it.
    for (std::size_t i = startIndex; i + 1 < pts_.size(); i++) {
        const double d0 = pts_[i].p.distTo(robot);
        const double d1 = pts_[i + 1].p.distTo(robot);
        if (d0 <= lookahead && d1 >= lookahead) {
            const double t = (lookahead - d0) / std::max(d1 - d0, 1e-9);
            const Point& a = pts_[i].p;
            const Point& b = pts_[i + 1].p;
            return {a.x + (b.x - a.x) * clamp(t, 0.0, 1.0),
                    a.y + (b.y - a.y) * clamp(t, 0.0, 1.0)};
        }
    }

    // No crossing found (robot far off the path): aim at the closest point
    // plus a bit of forward progress so we rejoin smoothly.
    const std::size_t ahead = std::min(startIndex + std::size_t(lookahead / 1.0),
                                       pts_.size() - 1);
    return pts_[ahead].p;
}

} // namespace aklib
