#include "geometry.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>

namespace uwb {
namespace {

std::string trim(const std::string& s)
{
    const auto a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    return s.substr(a, s.find_last_not_of(" \t\r\n") - a + 1);
}

// Gaussian elimination with partial pivoting; a is n x n.
std::optional<std::vector<double>> solve_linear(std::vector<std::vector<double>> a, std::vector<double> b)
{
    const size_t n = b.size();
    for (size_t i = 0; i < n; ++i) a[i].push_back(b[i]);
    for (size_t c = 0; c < n; ++c) {
        size_t p = c;
        for (size_t r = c + 1; r < n; ++r)
            if (std::fabs(a[r][c]) > std::fabs(a[p][c])) p = r;
        if (std::fabs(a[p][c]) < 1e-12) return std::nullopt;
        std::swap(a[c], a[p]);
        for (size_t r = c + 1; r < n; ++r) {
            const double f = a[r][c] / a[c][c];
            for (size_t k = c; k <= n; ++k) a[r][k] -= f * a[c][k];
        }
    }
    std::vector<double> x(n, 0.0);
    for (size_t ri = n; ri-- > 0;) {
        double s = a[ri][n];
        for (size_t k = ri + 1; k < n; ++k) s -= a[ri][k] * x[k];
        x[ri] = s / a[ri][ri];
    }
    return x;
}

}  // namespace

bool AnchorMap::load(const std::string& path, std::string& err)
{
    std::ifstream f(path);
    if (!f) {
        err = "cannot open " + path;
        return false;
    }
    file_anchors_.clear();
    bool in_anchors = false;
    std::string line;
    while (std::getline(f, line)) {
        if (const auto h = line.find('#'); h != std::string::npos) line.erase(h);
        line = trim(line);
        if (line.empty()) continue;
        if (line == "[anchors]") {
            in_anchors = true;
            continue;
        }
        if (line.front() == '[') {
            in_anchors = false;
            continue;
        }
        const auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = trim(line.substr(0, eq)), val = trim(line.substr(eq + 1));
        try {
            if (in_anchors) {
                const auto lb = val.find('['), comma = val.find(','), rb = val.find(']');
                if (lb == std::string::npos || comma == std::string::npos || rb == std::string::npos) continue;
                file_anchors_[std::stoi(key)] = {std::stod(val.substr(lb + 1, comma - lb - 1)),
                                                 std::stod(val.substr(comma + 1, rb - comma - 1))};
            } else if (key == "source") {
                source_ = trim(val);
                source_.erase(std::remove(source_.begin(), source_.end(), '"'), source_.end());
            } else if (key == "anchor_height") {
                anchor_height = std::stod(val);
            } else if (key == "tag_height") {
                file_tag_height = std::stod(val);
            }
        } catch (const std::exception&) {
            err = "bad line in " + path + ": " + line;
            return false;
        }
    }
    if (file_anchors_.size() < 3) {
        err = path + ": need at least 3 anchors in [anchors]";
        return false;
    }
    return true;
}

double horizontal(double range_m, double anchor_h, double tag_h)
{
    const double dz = anchor_h - tag_h;
    return std::sqrt(std::max(range_m * range_m - dz * dz, 0.0));
}

std::optional<Fix2D> solve_2d(const std::vector<Vec2>& pts, const std::vector<double>& dist, Vec2 guess,
                              const std::vector<double>& sigma)
{
    if (pts.size() < 3 || pts.size() != dist.size() || (!sigma.empty() && sigma.size() != pts.size())) return std::nullopt;
    double x = guess.x, y = guess.y;
    for (int it = 0; it < 15; ++it) {
        double a11 = 0, a12 = 0, a22 = 0, b1 = 0, b2 = 0;
        for (size_t i = 0; i < pts.size(); ++i) {
            const double dx = x - pts[i].x, dy = y - pts[i].y;
            double r = std::hypot(dx, dy);
            if (r == 0) r = 1e-6;
            const double w = sigma.empty() ? 1.0 : 1.0 / (sigma[i] * sigma[i]);
            const double jx = dx / r, jy = dy / r, res = r - dist[i];
            a11 += w * jx * jx;
            a12 += w * jx * jy;
            a22 += w * jy * jy;
            b1 -= w * jx * res;
            b2 -= w * jy * res;
        }
        const double det = a11 * a22 - a12 * a12;
        if (std::fabs(det) < 1e-12 * (a11 + a22) * (a11 + a22)) return std::nullopt;
        const double sx = (a22 * b1 - a12 * b2) / det, sy = (a11 * b2 - a12 * b1) / det;
        x += sx;
        y += sy;
        if (std::fabs(sx) + std::fabs(sy) < 1e-4) break;
    }
    double ss = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        const double e = std::hypot(x - pts[i].x, y - pts[i].y) - dist[i];
        ss += e * e;
    }
    if (!std::isfinite(x) || !std::isfinite(y)) return std::nullopt;
    return Fix2D{x, y, std::sqrt(ss / static_cast<double>(pts.size()))};
}

std::optional<Fix3D> solve_3d(const std::vector<Vec2>& pts, const std::vector<double>& range, Vec2 guess, double dz_guess)
{
    if (pts.size() < 4 || pts.size() != range.size()) return std::nullopt;
    double x = guess.x, y = guess.y, d = std::max(dz_guess, 0.3);
    for (int it = 0; it < 20; ++it) {
        std::vector<std::vector<double>> a(3, std::vector<double>(3, 0.0));
        std::vector<double> b(3, 0.0);
        for (size_t i = 0; i < pts.size(); ++i) {
            const double dx = x - pts[i].x, dy = y - pts[i].y;
            const double r = std::max(std::sqrt(dx * dx + dy * dy + d * d), 1e-6);
            const double j[3] = {dx / r, dy / r, d / r};
            const double res = r - range[i];
            for (int p = 0; p < 3; ++p) {
                b[p] -= j[p] * res;
                for (int q = 0; q < 3; ++q) a[p][q] += j[p] * j[q];
            }
        }
        for (int p = 0; p < 3; ++p) a[p][p] += 1e-6;  // keeps the step finite when |dz| is barely observable
        const auto st = solve_linear(a, b);
        if (!st) return std::nullopt;
        x += (*st)[0];
        y += (*st)[1];
        d = std::max(d + (*st)[2], 0.02);  // the tag is on one side of the anchor plane; only |dz| matters
        if (std::fabs((*st)[0]) + std::fabs((*st)[1]) + std::fabs((*st)[2]) < 1e-4) break;
    }
    double ss = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        const double e = std::sqrt(std::pow(x - pts[i].x, 2) + std::pow(y - pts[i].y, 2) + d * d) - range[i];
        ss += e * e;
    }
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(d)) return std::nullopt;
    return Fix3D{x, y, d, std::sqrt(ss / static_cast<double>(pts.size()))};
}

std::optional<Cov2> position_covariance(const std::vector<Vec2>& pts, Vec2 pos, const std::vector<double>& sigma)
{
    if (sigma.size() != pts.size()) return std::nullopt;
    double a11 = 0, a12 = 0, a22 = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        const double dx = pos.x - pts[i].x, dy = pos.y - pts[i].y;
        const double r = std::max(std::hypot(dx, dy), 1e-6);
        const double hx = dx / r, hy = dy / r, w = 1.0 / (sigma[i] * sigma[i]);
        a11 += w * hx * hx;
        a12 += w * hx * hy;
        a22 += w * hy * hy;
    }
    const double det = a11 * a22 - a12 * a12;
    if (std::fabs(det) < 1e-12) return std::nullopt;
    return Cov2{a22 / det, a11 / det};
}

std::optional<Layout> solve_layout(const Pairs& pairs)
{
    std::set<int> idset;
    for (const auto& [k, v] : pairs) {
        (void)v;
        idset.insert(k.first);
        idset.insert(k.second);
    }
    const std::vector<int> ids(idset.begin(), idset.end());
    if (ids.size() < 3) return std::nullopt;
    auto dist = [&](int a, int b) -> std::optional<double> {
        const auto it = pairs.find({std::min(a, b), std::max(a, b)});
        if (it == pairs.end()) return std::nullopt;
        return it->second;
    };
    const int a1 = ids[0], a2 = ids[1];
    const auto d12 = dist(a1, a2);
    if (!d12 || *d12 <= 0) return std::nullopt;

    // Initial guess: A1 at the origin, A2 on +x, each further anchor trilaterated from two placed ones,
    // taking the solution on the +y side. Works with one pair missing (5 of 6 for four anchors).
    std::map<int, Vec2> pos;
    pos[a1] = {0, 0};
    pos[a2] = {*d12, 0};
    std::vector<int> pending(ids.begin() + 2, ids.end());
    while (!pending.empty()) {
        bool progressed = false;
        for (auto it = pending.begin(); it != pending.end();) {
            const int k = *it;
            std::vector<int> refs;
            for (const auto& [p, v] : pos) {
                (void)v;
                if (dist(p, k)) refs.push_back(p);
            }
            if (refs.size() < 2) {
                ++it;
                continue;
            }
            const int p = refs[0], q = refs[1];
            const Vec2 P = pos[p], Q = pos[q];
            const double base = std::hypot(Q.x - P.x, Q.y - P.y);
            if (base < 1e-6) {
                ++it;
                continue;
            }
            const double ex = (Q.x - P.x) / base, ey = (Q.y - P.y) / base;
            const double dpk = *dist(p, k), dqk = *dist(q, k);
            const double along = (dpk * dpk - dqk * dqk + base * base) / (2 * base);
            double off = std::sqrt(std::max(dpk * dpk - along * along, 0.0));
            if (off == 0) off = 1e-3;
            const Vec2 c1{P.x + along * ex - off * ey, P.y + along * ey + off * ex};
            const Vec2 c2{P.x + along * ex + off * ey, P.y + along * ey - off * ex};
            pos[k] = (c1.y >= c2.y) ? c1 : c2;
            it = pending.erase(it);
            progressed = true;
        }
        if (!progressed) return std::nullopt;
    }

    // Gauss-Newton over every pair. Unknowns: x of A2, then (x, y) of the others.
    std::vector<std::pair<int, int>> params{{a2, 0}};
    for (size_t i = 2; i < ids.size(); ++i) {
        params.push_back({ids[i], 0});
        params.push_back({ids[i], 1});
    }
    auto index_of = [&](int node, int c) -> int {
        for (size_t i = 0; i < params.size(); ++i)
            if (params[i].first == node && params[i].second == c) return static_cast<int>(i);
        return -1;
    };
    const size_t np = params.size();
    for (int iter = 0; iter < 30; ++iter) {
        std::vector<std::vector<double>> jtj(np, std::vector<double>(np, 0.0));
        std::vector<double> jtr(np, 0.0);
        for (const auto& [e, meas] : pairs) {
            const int a = e.first, b = e.second;
            if (!pos.count(a) || !pos.count(b)) continue;
            const Vec2 pa = pos[a], pb = pos[b];
            double r = std::hypot(pa.x - pb.x, pa.y - pb.y);
            if (r == 0) r = 1e-9;
            const double res = r - meas;
            std::vector<std::pair<int, double>> grad;
            for (const auto& [node, sign] : {std::pair<int, double>{a, 1.0}, std::pair<int, double>{b, -1.0}}) {
                const double ux = sign * (pa.x - pb.x) / r, uy = sign * (pa.y - pb.y) / r;
                if (const int i = index_of(node, 0); i >= 0) grad.push_back({i, ux});
                if (const int i = index_of(node, 1); i >= 0) grad.push_back({i, uy});
            }
            for (const auto& [i, gi] : grad) {
                jtr[static_cast<size_t>(i)] -= gi * res;
                for (const auto& [j, gj] : grad) jtj[static_cast<size_t>(i)][static_cast<size_t>(j)] += gi * gj;
            }
        }
        const auto step = solve_linear(jtj, jtr);
        if (!step) break;
        double biggest = 0;
        for (size_t i = 0; i < np; ++i) {
            auto& p = pos[params[i].first];
            (params[i].second == 0 ? p.x : p.y) += (*step)[i];
            biggest = std::max(biggest, std::fabs((*step)[i]));
        }
        if (biggest < 1e-6) break;
    }

    Layout lay;
    lay.anchors = pos;
    lay.n_pairs = static_cast<int>(pairs.size());
    lay.redundancy = lay.n_pairs - static_cast<int>(np);
    double ss = 0;
    for (const auto& [e, meas] : pairs) {
        const double solved = std::hypot(pos[e.first].x - pos[e.second].x, pos[e.first].y - pos[e.second].y);
        ss += (meas - solved) * (meas - solved) * 1e6;
    }
    lay.rms_mm = std::sqrt(ss / static_cast<double>(pairs.size()));
    return lay;
}

}  // namespace uwb
