// uwb_bridge: UWB tag -> 2D position -> VISION_POSITION_ESTIMATE -> ArduPilot, through fc_router.
//
//   uwb_bridge --heading 196 --origin 21.0285,105.8542,10 --tag-z 0.10 [--anchors config/anchors.toml]
//
// Normally started by scripts/run_uwb.sh (and by the dl-uwb systemd service at boot).
//
// Timing, which is what ArduPilot's EKF3 is picky about:
//   - one message every 1/--rate s on an absolute schedule (sleep_until, no drift), default 30 Hz
//   - each message is stamped with the mean arrival time of the fixes in its window, not the send time
//   - EKF3 drops any sample closer than 20 ms to the previous accepted one, so a window whose stamp
//     is less than --min-interval-ms (21) after the last one is merged into the next window instead
//   - nothing is sent while fewer than 3 anchors answer, or before the FC checks (attitude, height
//     source) pass, so the autopilot sees the loss and fails safe
#include <algorithm>
#include <atomic>
#include <csignal>
#include <format>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <deque>
#include <tuple>
#include <ctime>
#include <optional>
#include <print>
#include <string>
#include <thread>

#include "fc_link.hpp"
#include "geometry.hpp"
#include "tag_tracker.hpp"

using namespace std::chrono_literals;
using uwb::Clock;

namespace {

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop = true; }

struct Options {
    std::string fc_url = "udpout://127.0.0.1:14540";
    uint32_t sysid = 1;
    int compid = 197;
    double heading = NAN;  // compass bearing of the UWB +x axis (A1 -> A2), degrees
    bool have_origin = false;
    double lat = 0, lon = 0, alt = 0;
    double tag_z = 0.10;
    double rate = 30.0;
    double min_interval_ms = 21.0;
    double sigma = 0.05;
    double range_offset = 0.0;
    std::string tag_port;
    std::string anchors = "config/anchors.toml";
    std::string raw_dir;
    double vmax = 4.0;
    bool require_survey = false;
    double dz_max = 3.0;
    bool dry_run = false;
};

void usage(const char* argv0)
{
    std::println(stderr,
                 "Usage: {} --heading DEG [--origin LAT,LON,ALT] [--tag-z M] [--rate HZ] [--min-interval-ms MS]\n"
                 "          [--anchors FILE] [--tag-port DEV] [--fc-url URL] [--sysid N] [--compid N]\n"
                 "          [--sigma-range M] [--range-offset M] [--raw-dir DIR] [--vmax M/S] [--dry-run]\n"
                 "  --heading   compass bearing of the UWB x axis (A1 -> A2); not the drone's yaw\n"
                 "  --origin    EKF origin of the flying site (needed unless --dry-run)\n"
                 "  --tag-z     tag height above the floor when the drone sits on the ground (m); used until UWB measures it\n"
                 "  --raw-dir   write every range and fix to DIR/uwb_raw_*.csv (newest 30 kept)\n"
                 "  --vmax      fastest plausible horizontal speed for the jump gate (default 4 m/s)\n"
                 "  --rate      messages per second (default 30; EKF3 drops samples closer than 20 ms)\n"
                 "  --require-survey  use only the tag's survey of this session (anchors move between sessions); while\n"
                 "              disarmed and the layout is incomplete the tag is asked to measure the missing pairs\n"
                 "  --dz-max    largest plausible anchor-plane-to-tag distance (default 3 m)\n"
                 "  --dry-run   read the tag and print statistics, do not connect to the FC",
                 argv0);
}

Options parse(int argc, char** argv)
{
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument("missing value for " + a);
            return argv[++i];
        };
        if (a == "--heading") o.heading = std::stod(next());
        else if (a == "--origin") {
            const std::string v = next();
            const auto c1 = v.find(','), c2 = v.find(',', c1 + 1);
            if (c1 == std::string::npos || c2 == std::string::npos) throw std::invalid_argument("--origin LAT,LON,ALT");
            o.lat = std::stod(v.substr(0, c1));
            o.lon = std::stod(v.substr(c1 + 1, c2 - c1 - 1));
            o.alt = std::stod(v.substr(c2 + 1));
            o.have_origin = true;
        } else if (a == "--tag-z") o.tag_z = std::stod(next());
        else if (a == "--rate") o.rate = std::stod(next());
        else if (a == "--min-interval-ms") o.min_interval_ms = std::stod(next());
        else if (a == "--sigma-range") o.sigma = std::stod(next());
        else if (a == "--range-offset") o.range_offset = std::stod(next());
        else if (a == "--tag-port") o.tag_port = next();
        else if (a == "--anchors") o.anchors = next();
        else if (a == "--raw-dir") o.raw_dir = next();
        else if (a == "--vmax") o.vmax = std::stod(next());
        else if (a == "--require-survey") o.require_survey = true;
        else if (a == "--dz-max") o.dz_max = std::stod(next());
        else if (a == "--fc-url") o.fc_url = next();
        else if (a == "--sysid") o.sysid = static_cast<uint32_t>(std::stoi(next()));
        else if (a == "--compid") o.compid = std::stoi(next());
        else if (a == "--dry-run") o.dry_run = true;
        else if (a == "--help" || a == "-h") throw std::invalid_argument("");
        else throw std::invalid_argument("unknown option " + a);
    }
    if (std::isnan(o.heading)) throw std::invalid_argument("--heading is required (compass bearing of A1 -> A2; measure with scripts/measure_heading.sh)");
    if (!o.dry_run && !o.have_origin) throw std::invalid_argument("--origin LAT,LON,ALT is required (ArduPilot has no GPS to take it from)");
    if (o.rate <= 0 || o.rate > 50) throw std::invalid_argument("--rate must be in (0, 50]");
    if (o.min_interval_ms < 20.0) throw std::invalid_argument("--min-interval-ms below 20 would let EKF3 drop samples");
    return o;
}

// Running min / max / mean / sigma of a series of values.
struct Series {
    double lo = 1e18, hi = -1e18, sum = 0, sum2 = 0;
    int n = 0;
    void add(double v)
    {
        lo = std::min(lo, v);
        hi = std::max(hi, v);
        sum += v;
        sum2 += v * v;
        ++n;
    }
    double mean() const { return n ? sum / n : 0; }
    double sigma() const { return n > 1 ? std::sqrt(std::max(sum2 / n - mean() * mean(), 0.0)) : 0; }
    void reset() { *this = Series{}; }
};

std::string clock_str()
{
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char b[16];
    std::strftime(b, sizeof b, "%H:%M:%S", &tm);
    return b;
}

}  // namespace

int main(int argc, char** argv)
{
    std::setvbuf(stdout, nullptr, _IOLBF, 0);  // line-buffered so journald and logs show lines at once
    Options opt;
    try {
        opt = parse(argc, argv);
    } catch (const std::exception& e) {
        if (*e.what()) std::println(stderr, "{}", e.what());
        usage(argv[0]);
        return 2;
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    uwb::AnchorMap map;
    if (std::string err; !map.load(opt.anchors, err)) {
        std::println(stderr, "FAIL: {}", err);
        return 1;
    }
    const bool file_is_survey_only = map.auto_source();
    std::println("uwb_bridge: x axis heading {:.1f} deg | rate {:g} Hz | min spacing {:g} ms | tag_z {:.2f} m | anchor_height {:.2f} m | anchors {} ({}){}",
                 opt.heading, opt.rate, opt.min_interval_ms, opt.tag_z, map.anchor_height, opt.anchors,
                 opt.require_survey ? "chỉ dùng bản đồ tag tự calib" : file_is_survey_only ? "file until the tag's survey completes" : "file only",
                 opt.dry_run ? " | DRY RUN" : "");

    uwb::TagTracker::Options to;
    to.port = opt.tag_port;
    to.sigma = opt.sigma;
    to.range_offset = opt.range_offset;
    to.raw_dir = opt.raw_dir;
    to.vmax = opt.vmax;
    to.require_survey = opt.require_survey;
    to.dz_max = opt.dz_max;
    uwb::TagTracker tracker(std::move(map), to);
    // The geometry takes the tag height from UWB itself (3D solve over 4 anchors). The FC's baro height is only
    // printed for comparison: in flight prop wash moved it by metres, which became metres of horizontal error.
    tracker.set_tag_height(opt.tag_z);
    tracker.start();

    uwb::FcLink fc;
    if (!opt.dry_run) {
        uwb::FcLink::Config fcfg;
        fcfg.url = opt.fc_url;
        fcfg.sysid = opt.sysid;
        fcfg.compid = static_cast<uint8_t>(opt.compid);
        fcfg.have_origin = opt.have_origin;
        fcfg.lat = opt.lat;
        fcfg.lon = opt.lon;
        fcfg.alt = opt.alt;
        if (!fc.start(fcfg, g_stop)) {
            tracker.stop();
            return g_stop ? 0 : 1;
        }
    }

    const double psi = opt.heading * M_PI / 180.0;
    const double c = std::cos(psi), s = std::sin(psi);
    const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / opt.rate));
    const auto min_interval = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double, std::milli>(opt.min_interval_ms));
    const auto t_origin = Clock::now();

    auto next = Clock::now() + period;
    auto next_maint = Clock::now();
    auto next_print = Clock::now() + 2s;
    auto last_print = Clock::now();
    std::vector<uwb::Fix> pending;
    std::optional<Clock::time_point> last_stamp, last_send;
    double h0 = NAN, fc_h = NAN;
    std::optional<std::pair<double, double>> last_ne;

    uint64_t sent = 0, empty = 0, merged = 0, late = 0, resync = 0, blocked = 0;

    // Anchor map supervision. On the ground: ask the tag for the missing survey pairs, and refuse to feed the EKF
    // while most fixes disagree with the layout (anchors moved, wrong map), so the FC reports VisOdom unhealthy
    // and does not arm. In flight nothing is requested or blocked.
    auto next_map_check = Clock::now();
    std::deque<std::tuple<Clock::time_point, uint64_t, uint64_t>> fix_hist;
    bool map_bad = false;
    std::optional<Clock::time_point> bad_since;
    Clock::time_point last_more{}, last_full{};
    uint64_t prev_ok = 0, prev_fail = 0, prev_rej = 0, prev_gate = 0, prev_drop = 0, prev_nlos = 0;
    double last_sigma = 0;
    Series gap_send, gap_stamp;
    int low_rate_streak = 0;

    while (!g_stop) {
        std::this_thread::sleep_until(next);
        const auto now = Clock::now();
        if (std::chrono::duration<double, std::milli>(now - next).count() > 5.0) ++late;  // wake-up latency
        next += period;
        if (now > next) {  // more than a whole period behind (suspend, stall): restart the schedule, no burst
            next = now + period;
            ++resync;
        }

        if (!opt.dry_run && now >= next_maint) {
            fc.maintain();
            next_maint = now + 1s;
        }

        const bool armed = !opt.dry_run && fc.armed();
        tracker.set_layout_locked(armed);
        if (!opt.dry_run) {
            if (const auto h = fc.height()) {
                if (!armed) h0 = std::isnan(h0) ? *h : 0.98 * h0 + 0.02 * *h;
                fc_h = *h - (std::isnan(h0) ? 0.0 : h0) + opt.tag_z;  // display only
            }
        }

        if (now >= next_map_check) {
            next_map_check = now + 500ms;
            const auto ms = tracker.stats();
            fix_hist.emplace_back(now, ms.fixes, ms.rejected);
            while (fix_hist.size() > 1 && now - std::get<0>(fix_hist.front()) > 4s) fix_hist.pop_front();
            const uint64_t df = ms.fixes - std::get<1>(fix_hist.front()), dr = ms.rejected - std::get<2>(fix_hist.front());
            const bool inconsistent = df + dr >= 20 && dr > df;  // over ~4 s more fixes rejected than accepted
            map_bad = !armed && ms.map_ready && inconsistent;
            if (map_bad && !bad_since) {
                bad_since = now;
                std::println("[bản đồ] khoảng cách đo không khớp với bản đồ anchor ({} fix bị loại / {} hợp lệ trong 4 s): chặn gửi vị trí", dr, df);
            }
            if (!map_bad) bad_since.reset();
            const int state = ms.has_status && ms.status.has_v02 ? ms.status.survey_state : -1;
            const int need = ms.has_status && ms.status.n_anchors >= 3 ? 2 * ms.status.n_anchors - 3 : 5;
            if (!armed && ms.port_open) {
                // 3 = incomplete (tag retries by itself), 5 = gave up; ask for more only when the tag is idle
                if (state == 5 && ms.host_pairs < need && now - last_more > 10s) {
                    last_more = now;
                    tracker.request_survey_more();
                }
                if (bad_since && now - *bad_since > 15s && now - last_full > 60s) {
                    last_full = now;
                    std::println("[bản đồ] vẫn không khớp sau 15 s: yêu cầu tag đo lại toàn bộ (anchor đã bị dời?)");
                    tracker.request_resurvey();
                }
            }
        }

        // One window per tick: everything that arrived since the last tick, plus anything merged earlier.
        auto got = tracker.take();
        pending.insert(pending.end(), got.begin(), got.end());
        std::erase_if(pending, [&](const uwb::Fix& f) { return now - f.t > 150ms; });  // never send a stale position
        const bool may_send = (opt.dry_run || fc.ready()) && !map_bad;
        if (!may_send) {
            if (!pending.empty()) ++blocked;
            pending.clear();
        } else if (pending.empty()) {
            ++empty;
        } else {
            double ts = 0, x = 0, y = 0, vx = 0, vy = 0;
            int n_anchors = 99;
            for (const auto& f : pending) {
                ts += std::chrono::duration<double>(f.t - t_origin).count();
                x += f.x;
                y += f.y;
                vx += f.vx;
                vy += f.vy;
                n_anchors = std::min(n_anchors, f.n_anchors);
            }
            const double k = static_cast<double>(pending.size());
            const auto stamp = t_origin + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(ts / k));
            if (last_stamp && stamp - *last_stamp < min_interval) {
                ++merged;  // EKF3 would drop it: keep the fixes for the next window
            } else {
                x /= k;
                y /= k;
                vx /= k;  // fixes in one window share most of their ranges: keep the mean variance, not /k^2
                vy /= k;
                // UWB (x towards A2, y towards A4, counter-clockwise) -> NED, origin at A1
                const double north = x * c + y * s, east = x * s - y * c;
                const double var_n = vx * c * c + vy * s * s, var_e = vx * s * s + vy * c * c;
                const auto usec = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(stamp - t_origin).count());
                const bool ok = opt.dry_run ? true : fc.send_vision_position(usec, north, east, var_n, var_e);
                if (ok) {
                    ++sent;
                    last_ne = {north, east};
                    last_sigma = std::sqrt(var_n + var_e);
                    if (last_send) gap_send.add(std::chrono::duration<double, std::milli>(now - *last_send).count());
                    if (last_stamp) gap_stamp.add(std::chrono::duration<double, std::milli>(stamp - *last_stamp).count());
                    last_send = now;
                    last_stamp = stamp;
                }
                pending.clear();
            }
        }

        if (now >= next_print) {
            const double dt = std::chrono::duration<double>(now - last_print).count();
            last_print = now;
            next_print = now + 2s;
            const auto ts = tracker.stats();
            const double hz = static_cast<double>(sent) / dt;
            std::string fcs = "FC: chế độ thử (không nối)";
            if (!opt.dry_run) {
                fcs = std::format("FC: {} origin {} | độ cao: {}", fc.connected() ? (fc.armed() ? "ARMED" : "disarmed") : "chưa kết nối",
                                  fc.origin_set() ? "OK" : "chưa có",
                                  fc.z_state() == uwb::FcLink::ZState::Ok        ? "không lấy từ UWB (" + fc.z_summary() + ")"
                                  : fc.z_state() == uwb::FcLink::ZState::Blocked ? "ĐANG CHẶN GỬI (" + fc.z_summary() + ")"
                                                                                  : "chưa kiểm tra");
            }
            const int all_pairs = ts.has_status && ts.status.n_anchors ? ts.status.n_anchors * (ts.status.n_anchors - 1) / 2 : 6;
            std::string survey = !ts.map_ready     ? std::format("CHƯA CÓ, đang calib ({}/{} cặp)", ts.host_pairs, all_pairs)
                                 : ts.using_survey ? std::format("tự calib ({}/{} cặp, sai số dư {:.0f} mm)", ts.host_pairs, all_pairs, ts.survey_rms_mm)
                                                   : std::string("anchors.toml");
            if (map_bad) survey += " ⚠ KHÔNG KHỚP, chặn gửi";
            if (ts.has_status && ts.status.has_v02) survey += std::format(", tag: {}", uwb::survey_state_name(ts.status.survey_state));
            std::string pos = last_ne ? std::format("N {:+.2f} E {:+.2f} m", last_ne->first, last_ne->second) : "chưa có vị trí";
            std::println("[{}] gửi {:4.1f} Hz (đích {:g}) | gói cách {:.1f} ms [{:.1f}..{:.1f}] σ {:.1f}, mẫu cách min {:.1f} ms | {} | đo ok {:.0f}/s lỗi {:.0f}/s | "
                         "anchor {} | trống {} gộp {} trễ {} chặn {} | lọc: bỏ {:.0f}/s nhảy {:.0f}/s bớt-1-anchor {:.0f}/s NLOS {:.0f}/s, σ gửi {:.2f} m | "
                         "tag cao UWB {} | FC {} | bản đồ {} | {}{}",
                         clock_str(), hz, opt.rate, gap_send.mean(), gap_send.n ? gap_send.lo : 0.0, gap_send.n ? gap_send.hi : 0.0, gap_send.sigma(),
                         gap_stamp.n ? gap_stamp.lo : 0.0, pos, static_cast<double>(ts.ok - prev_ok) / dt, static_cast<double>(ts.fail - prev_fail) / dt,
                         ts.anchors_fresh, empty, merged, late, blocked, static_cast<double>(ts.rejected - prev_rej) / dt,
                         static_cast<double>(ts.gated - prev_gate) / dt, static_cast<double>(ts.dropped_anchor - prev_drop) / dt,
                         static_cast<double>(ts.nlos - prev_nlos) / dt, last_sigma,
                         std::isnan(ts.tag_height) ? std::string("chưa đo (dùng tag_z)") : std::format("{:.2f} m{}", ts.tag_height, ts.height_from_uwb ? "" : " (giữ giá trị cũ)"),
                         std::isnan(fc_h) ? std::string("-") : std::format("{:.2f} m", fc_h), survey, fcs, ts.port_open ? "" : " | ⚠ không thấy tag");
            const bool flying_ok = opt.dry_run || fc.ready();
            low_rate_streak = (flying_ok && hz < 0.85 * opt.rate) ? low_rate_streak + 1 : 0;
            if (low_rate_streak == 3)
                std::println("[cảnh báo] tần số gửi {:.1f} Hz thấp hơn 85% mức đích: ít hơn 3 anchor trả lời đủ nhanh (A2/A4 yếu?) hoặc tag mất kết nối.", hz);
            if (resync) std::println("[cảnh báo] vòng gửi bị chậm hơn một chu kỳ {} lần (hệ thống bận?)", resync);
            prev_ok = ts.ok;
            prev_fail = ts.fail;
            prev_rej = ts.rejected;
            prev_gate = ts.gated;
            prev_drop = ts.dropped_anchor;
            prev_nlos = ts.nlos;
            sent = empty = merged = late = resync = blocked = 0;
            gap_send.reset();
            gap_stamp.reset();
        }
    }
    std::println("uwb_bridge: dừng");
    tracker.stop();
    return 0;
}
