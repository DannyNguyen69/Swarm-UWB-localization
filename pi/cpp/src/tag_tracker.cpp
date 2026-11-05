#include "tag_tracker.hpp"

#include <glob.h>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <format>
#include <print>

namespace uwb {

namespace {
double secs(Clock::duration d) { return std::chrono::duration<double>(d).count(); }
}  // namespace

void TagTracker::start()
{
    stop_ = false;
    open_raw();
    th_ = std::thread([this] { run(); });
}

void TagTracker::stop()
{
    stop_ = true;
    if (th_.joinable()) th_.join();
    if (raw_) {
        std::fclose(raw_);
        raw_ = nullptr;
    }
}

// logs/uwb_raw_YYYYmmdd_HHMMSS.csv, the newest 30 are kept. Two record types:
//   R,t_s,seq,anchor,range_mm,rx_dbm,fp_dbm,flags        every range the tag reports
//   F,t_s,x,y,dz,rms,n_used,state                        every fix; state 0 ok, 1 dropped one anchor, 2 jump gate, 3 rejected
//   P,t_s,table_version,a,b,dist_mm,std_mm,n_samples     every anchor pair the tag reports (survey)
//   S,t_s,alive_mask,pairs_valid,survey_state            tag status, when it changes
void TagTracker::open_raw()
{
    if (opt_.raw_dir.empty()) return;
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::create_directories(opt_.raw_dir, ec);
    std::vector<fs::path> old;
    for (const auto& e : fs::directory_iterator(opt_.raw_dir, ec))
        if (e.path().filename().string().starts_with("uwb_raw_")) old.push_back(e.path());
    std::sort(old.begin(), old.end());
    for (size_t i = 0; i + 29 < old.size(); ++i) fs::remove(old[i], ec);

    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char name[64];
    std::strftime(name, sizeof name, "uwb_raw_%Y%m%d_%H%M%S.csv", &tm);
    const std::string path = (fs::path(opt_.raw_dir) / name).string();
    raw_ = std::fopen(path.c_str(), "w");
    if (!raw_) {
        std::println("[tag] không ghi được {}", path);
        return;
    }
    std::fprintf(raw_, "# R,t_s,seq,anchor,range_mm,rx_dbm,fp_dbm,flags | F,t_s,x,y,dz,rms,n_used,state(0 ok,1 drop1,2 gate,3 reject) | P,t_s,ver,a,b,dist_mm,std_mm,n | S,t_s,alive,pairs,state\n");
    {
        std::lock_guard lk(mu_);
        raw_name_ = path;
    }
    std::println("[tag] ghi khoảng cách thô vào {}", path);
}

void TagTracker::raw_line(const std::string& line)
{
    if (!raw_) return;
    std::fputs(line.c_str(), raw_);
    const auto now = Clock::now();
    if (now - raw_flush_ > std::chrono::seconds(1)) {
        std::fflush(raw_);
        raw_flush_ = now;
    }
}

std::string TagTracker::find_port(std::string& note)
{
    glob_t g{};
    std::string found;
    if (glob("/dev/serial/by-id/usb-SEGGER_J-Link_*-if00", 0, nullptr, &g) == 0) {
        if (g.gl_pathc == 1) found = g.gl_pathv[0];
        else note = std::to_string(g.gl_pathc) + " J-Link trên USB, dùng --tag-port để chọn";
    } else {
        note = "chưa thấy tag (J-Link) trên USB";
    }
    globfree(&g);
    return found;
}

void TagTracker::run()
{
    SerialPort sp;
    Parser parser;
    uint8_t buf[4096];
    std::string last_note;
    while (!stop_) {
        if (!sp.is_open()) {
            std::string note;
            const std::string port = opt_.port.empty() ? find_port(note) : opt_.port;
            if (port.empty() || !sp.open(port)) {
                if (note.empty()) note = "không mở được " + port;
                if (note != last_note) std::println("[tag] {}; thử lại mỗi giây", note);
                last_note = note;
                port_open_ = false;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            std::println("[tag] mở {}", port);
            last_note.clear();
            port_open_ = true;
            parser = Parser{};
            latest_.clear();
        }
        const ssize_t n = sp.read(buf, sizeof buf, 50);
        if (n < 0) {
            std::println("[tag] mất cổng serial, mở lại");
            sp.close();
            port_open_ = false;
            continue;
        }
        if (const int c = cmd_.exchange(0)) {
            const auto frame = encode_frame(static_cast<uint8_t>(c));
            if (sp.write(frame.data(), frame.size()))
                std::println("[tag] gửi lệnh {}", c == CMD_RESURVEY ? "đo lại toàn bộ bản đồ anchor" : "đo tiếp các cặp anchor còn thiếu");
        }
        if (n == 0) continue;
        const auto now = Clock::now();
        for (const auto& pkt : parser.feed(buf, static_cast<size_t>(n))) handle(pkt, now);
        crc_ = parser.crc_errors();
    }
}

void TagTracker::handle(const Packet& pkt, Clock::time_point now)
{
    if (const auto* r = std::get_if<Range>(&pkt)) {
        on_range(*r, now);
    } else if (const auto* s = std::get_if<Status>(&pkt)) {
        if (raw_ && s->has_v02 &&
            (s->alive_mask != last_logged_status_.alive_mask || s->pairs_valid != last_logged_status_.pairs_valid ||
             s->survey_state != last_logged_status_.survey_state)) {
            raw_line(std::format("S,{:.4f},{},{},{}\n", secs(now - t0_), s->alive_mask, s->pairs_valid, s->survey_state));
            last_logged_status_ = *s;
        }
        std::lock_guard lk(mu_);
        status_ = *s;
        has_status_ = true;
    } else if (const auto* p = std::get_if<AnchorPair>(&pkt)) {
        on_pair(*p);
    }
}

void TagTracker::on_range(const Range& r, Clock::time_point now)
{
    const double ts = secs(now - t0_);
    if (raw_) raw_line(std::format("R,{:.4f},{},{},{},{:.1f},{:.1f},{}\n", ts, r.seq, r.anchor_id, r.range_mm, r.rx_dbm, r.fp_dbm, r.flags));
    if (!r.ok() || r.range_mm <= 0 || r.range_mm > 50000) {
        ++fail_;
        return;
    }
    ++ok_;
    const bool nlos = (r.flags & F_NLOS) != 0;
    if (nlos) ++nlos_;
    latest_[r.anchor_id] = {now, r.range_mm / 1000.0 + opt_.range_offset, nlos};
    const bool map_ready = !opt_.require_survey || map_.using_survey();
    map_ready_ = map_ready;
    if (!map_ready) {
        int fresh = 0;
        for (const auto& [id, m] : latest_) fresh += secs(now - m.t) <= opt_.fresh_s;
        fresh_ = fresh;
        return;  // no layout for this session yet: a stale file layout would give confident, wrong positions
    }

    const auto& anchors = map_.anchors();
    std::vector<Vec2> pts;
    std::vector<double> rng;
    std::vector<bool> nl;
    for (const auto& [id, m] : latest_) {
        const auto it = anchors.find(id);
        if (it == anchors.end() || secs(now - m.t) > opt_.fresh_s) continue;
        pts.push_back(it->second);
        rng.push_back(m.range);
        nl.push_back(m.nlos);
    }
    const size_t n = pts.size();
    fresh_ = static_cast<int>(n);
    if (n < 3) return;

    Vec2 guess{0, 0};
    if (have_last_) {
        guess = {last_x_, last_y_};
    } else {
        for (const auto& p : pts) {
            guess.x += p.x / static_cast<double>(n);
            guess.y += p.y / static_cast<double>(n);
        }
        guess.x += 0.1;
        guess.y += 0.1;
    }
    const double H = map_.anchor_height;

    // 1. Height from UWB: a 3D solve over >= 4 anchors, used only when it is self-consistent, then smoothed.
    if (n >= 4) {
        const double dz0 = dz_valid_ ? dz_est_ : std::max(H - tag_h_.load(), 0.3);
        const auto s3 = solve_3d(pts, rng, guess, dz0);
        if (s3 && s3->rms < 0.08 && s3->dz <= opt_.dz_max) {
            dz_est_ = dz_valid_ ? dz_est_ + 0.05 * (s3->dz - dz_est_) : s3->dz;
            dz_valid_ = true;
            dz_time_ = now;
        }
    }
    const bool dz_fresh = dz_valid_ && secs(now - dz_time_) < 3.0;
    const double dz = dz_valid_ ? dz_est_ : std::max(H - tag_h_.load(), 0.0);
    const double sigma_dz = dz_fresh ? 0.10 : 0.30;
    tag_h_uwb_ = dz_valid_ ? H - dz_est_ : NAN;  // shown as height above the floor, assuming the tag is below the anchors
    h_uwb_ = dz_fresh;

    // 2. Horizontal distances and their noise: dh/dr = r/h grows near an anchor, NLOS ranges count 3x.
    std::vector<double> hd(n), sg(n);
    for (size_t i = 0; i < n; ++i) {
        const double h = std::sqrt(std::max(rng[i] * rng[i] - dz * dz, 0.0));
        const double hh = std::max(h, 0.3);
        hd[i] = h;
        const double s_r = opt_.sigma * std::min(rng[i] / hh, 6.0) * (nl[i] ? 3.0 : 1.0);
        const double s_z = sigma_dz * std::min(dz / hh, 6.0);
        sg[i] = std::hypot(s_r, s_z);
    }

    // 3. Solve; with 4 anchors and a bad residual, drop the anchor that explains it.
    auto sol = solve_2d(pts, hd, guess, sg);
    std::vector<size_t> used(n);
    for (size_t i = 0; i < n; ++i) used[i] = i;
    int state = 0;
    auto reject = [&](int st) {
        ++(st == 2 ? gated_ : rejected_);
        if (raw_) raw_line(std::format("F,{:.4f},,,{:.3f},,{},{}\n", ts, dz, n, st));
    };
    if (!sol) {
        reject(3);
        return;
    }
    if (n >= 4 && sol->rms > 0.15) {
        std::optional<Fix2D> best;
        std::vector<size_t> best_used;
        for (size_t k = 0; k < n; ++k) {
            std::vector<Vec2> p2;
            std::vector<double> h2, s2;
            std::vector<size_t> u2;
            for (size_t i = 0; i < n; ++i)
                if (i != k) {
                    p2.push_back(pts[i]);
                    h2.push_back(hd[i]);
                    s2.push_back(sg[i]);
                    u2.push_back(i);
                }
            const auto s = solve_2d(p2, h2, guess, s2);
            if (s && (!best || s->rms < best->rms)) {
                best = s;
                best_used = u2;
            }
        }
        if (!best || best->rms > 0.08) {
            reject(3);
            return;
        }
        sol = best;
        used = best_used;
        state = 1;
        ++dropped_;
    } else if (n == 3 && sol->rms > 0.25) {
        reject(3);
        return;
    }

    // 4. Jump gate: faster than the vehicle can move means a bad fix, unless it persists.
    if (have_last_) {
        const double dt = secs(now - last_t_);
        const double jump = std::hypot(sol->x - last_x_, sol->y - last_y_);
        if (jump > 0.3 + opt_.vmax * dt) {
            if (!gating_) {
                gating_ = true;
                gate_since_ = now;
            }
            if (secs(now - gate_since_) < opt_.gate_hold_s) {
                reject(2);
                return;
            }
        }
    }
    gating_ = false;

    // 5. Covariance: geometry and per-anchor noise, inflated by the residual and when there is no redundancy.
    std::vector<Vec2> pu;
    std::vector<double> su;
    for (size_t i : used) {
        pu.push_back(pts[i]);
        su.push_back(sg[i]);
    }
    const auto cov = position_covariance(pu, {sol->x, sol->y}, su);
    if (!cov) {
        reject(3);
        return;
    }
    double k = std::max(1.0, sol->rms / (1.5 * opt_.sigma));
    k *= k;
    if (used.size() == 3) k *= 2.0;
    const double vx = std::max(cov->xx * k, 0.03 * 0.03), vy = std::max(cov->yy * k, 0.03 * 0.03);

    have_last_ = true;
    last_t_ = now;
    last_x_ = sol->x;
    last_y_ = sol->y;
    ++nfix_;
    if (raw_) raw_line(std::format("F,{:.4f},{:.3f},{:.3f},{:.3f},{:.3f},{},{}\n", ts, sol->x, sol->y, dz, sol->rms, used.size(), state));

    std::lock_guard lk(mu_);
    fixes_.push_back({now, sol->x, sol->y, vx, vy, static_cast<int>(used.size())});
    if (fixes_.size() > 1000) fixes_.erase(fixes_.begin(), fixes_.begin() + 500);  // main loop stalled
}

void TagTracker::on_pair(const AnchorPair& p)
{
    if (raw_)
        raw_line(std::format("P,{:.4f},{},{},{},{},{},{}\n", secs(Clock::now() - t0_), p.table_version, p.id_a, p.id_b, p.dist_mm, p.std_mm,
                             p.n_samples));
    if (locked_) return;  // armed: the layout must not change
    pairs_[{std::min<int>(p.id_a, p.id_b), std::max<int>(p.id_a, p.id_b)}] = p;

    // Pairs of the newest table version only: pairs invalidated by an anchor reboot keep an older version.
    Pairs cur;
    for (const auto& [k, v] : pairs_)
        if (v.table_version == p.table_version && v.n_samples > 0) cur[k] = v.dist_mm / 1000.0;
    host_pairs_ = static_cast<int>(cur.size());
    int n = 0;
    {
        std::lock_guard lk(mu_);
        if (has_status_) n = status_.n_anchors;
    }
    // 2n - 3 pairs fix a rigid 2D layout (5 of 6 for four anchors).
    const size_t need = n ? static_cast<size_t>(std::max(2 * n - 3, 3)) : 3;
    const bool same = survey_version_ == p.table_version && map_.survey() &&
                      cur.size() <= static_cast<size_t>(map_.survey()->n_pairs);
    if (same || cur.size() < need) return;
    auto lay = solve_layout(cur);
    if (!lay) return;
    std::print("[tag] bản đồ anchor tự calib (bảng v{}, {} cặp, sai số dư {:.1f} mm):", p.table_version, lay->n_pairs, lay->rms_mm);
    for (const auto& [id, v] : lay->anchors) std::print(" A{}=({:.3f},{:.3f})", id, v.x, v.y);
    std::println("");
    survey_rms_ = lay->rms_mm;
    map_.set_survey(std::move(*lay));
    survey_version_ = p.table_version;
    using_survey_ = map_.using_survey();
}

std::vector<Fix> TagTracker::take()
{
    std::lock_guard lk(mu_);
    std::vector<Fix> out;
    out.swap(fixes_);
    return out;
}

TagStats TagTracker::stats() const
{
    TagStats s;
    s.ok = ok_;
    s.fail = fail_;
    s.crc_errors = crc_;
    s.anchors_fresh = fresh_;
    s.port_open = port_open_;
    s.using_survey = using_survey_;
    s.survey_rms_mm = survey_rms_;
    s.fixes = nfix_;
    s.rejected = rejected_;
    s.gated = gated_;
    s.dropped_anchor = dropped_;
    s.nlos = nlos_;
    s.tag_height = tag_h_uwb_;
    s.map_ready = map_ready_;
    s.host_pairs = host_pairs_;
    s.height_from_uwb = h_uwb_;
    std::lock_guard lk(mu_);
    s.raw_file = raw_name_;
    s.has_status = has_status_;
    s.status = status_;
    return s;
}

}  // namespace uwb
