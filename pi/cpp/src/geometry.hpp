// Anchor layout and the 2D position solver (ports of host/anchors.toml handling, uwb_viewer.solve_2d and
// survey_solver.solve_layout).
#pragma once
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace uwb {

struct Vec2 {
    double x = 0, y = 0;
};
using Pairs = std::map<std::pair<int, int>, double>;  // (a < b) -> measured distance in metres

struct Layout {
    std::map<int, Vec2> anchors;
    double rms_mm = 0;
    int n_pairs = 0;
    int redundancy = 0;
};

// Anchor coordinates: the tag's survey if source = "auto" and one has been solved, else the file.
class AnchorMap {
public:
    // Reads the subset of TOML used by anchors.toml: source, anchor_height, tag_height, [anchors] N = [x, y].
    bool load(const std::string& path, std::string& err);

    const std::map<int, Vec2>& anchors() const { return survey_ ? survey_->anchors : file_anchors_; }
    bool using_survey() const { return survey_.has_value(); }
    const Layout* survey() const { return survey_ ? &*survey_ : nullptr; }
    bool auto_source() const { return source_ == "auto"; }
    void set_survey(Layout l)
    {
        if (auto_source()) survey_ = std::move(l);
    }

    double anchor_height = 0;  // m above the floor
    double file_tag_height = 0;

private:
    std::map<int, Vec2> file_anchors_;
    std::string source_ = "auto";
    std::optional<Layout> survey_;
};

// 3D range to horizontal distance for anchors at anchor_h and a tag at tag_h.
double horizontal(double range_m, double anchor_h, double tag_h);

struct Fix2D {
    double x, y, rms;
};
// Gauss-Newton least squares; needs >= 3 anchors. sigma (optional, one per anchor) weights each distance by 1/sigma^2;
// rms is the unweighted residual.
std::optional<Fix2D> solve_2d(const std::vector<Vec2>& pts, const std::vector<double>& dist, Vec2 guess,
                              const std::vector<double>& sigma = {});

struct Fix3D {
    double x, y, dz, rms;  // dz = vertical distance between the anchor plane and the tag (>= 0)
};
// All anchors at one height: solves x, y and |dz| from the 3D ranges. Needs >= 4 anchors to be over-determined.
std::optional<Fix3D> solve_3d(const std::vector<Vec2>& pts, const std::vector<double>& range, Vec2 guess, double dz_guess);

struct Cov2 {
    double xx, yy;
};
// (H^T W H)^-1 diagonal for unit-line-of-sight rows H, W = 1/sigma_i^2.
std::optional<Cov2> position_covariance(const std::vector<Vec2>& pts, Vec2 pos, const std::vector<double>& sigma);

// A1 at the origin, A2 on +x, the others on the +y side; least squares over every measured pair.
std::optional<Layout> solve_layout(const Pairs& pairs);

}  // namespace uwb
