// Reads the tag's UART stream in its own thread and turns every successful range into a 2D position fix.
//
// Per range:
//   - the tag height comes from UWB itself: with >= 4 anchors a 3D solve gives |dz| (anchor plane to tag), smoothed
//     and only taken from consistent solutions. The FC's baro height is not used for the geometry: in flight prop
//     wash moved it by metres, which turned into metres of horizontal error. Until UWB has a height, set_tag_height()
//     (the tag height on the ground) is used.
//   - outliers: with 4 anchors a bad residual tries leaving each anchor out; a fix that jumps faster than the
//     vehicle can move is dropped (for at most gate_hold_s, then the gate re-centres)
//   - the covariance follows the geometry, the 3D-to-horizontal amplification r/h, NLOS flags and the residual
//   - every range and fix can be written to a CSV in raw_dir for later analysis
#pragma once
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry.hpp"
#include "serial_port.hpp"
#include "uwb_proto.hpp"

namespace uwb {

using Clock = std::chrono::steady_clock;

struct Fix {
    Clock::time_point t;    // arrival time of the range that produced the fix
    double x = 0, y = 0;    // UWB frame, metres
    double vx = 0, vy = 0;  // position variances, m^2
    int n_anchors = 0;      // anchors used
};

struct TagStats {
    uint64_t ok = 0, fail = 0, crc_errors = 0;
    uint64_t fixes = 0, rejected = 0, gated = 0, dropped_anchor = 0, nlos = 0;
    int anchors_fresh = 0;
    bool port_open = false;
    bool using_survey = false;
    double survey_rms_mm = 0;
    bool has_status = false;
    Status status;
    bool map_ready = false;   // an anchor layout is in use (with require_survey: only the tag's survey counts)
    int host_pairs = 0;       // anchor pairs of the newest table version seen by the host
    double tag_height = NAN;  // m above the floor, from UWB (NAN until known)
    bool height_from_uwb = false;
    std::string raw_file;
};

class TagTracker {
public:
    struct Options {
        std::string port;            // empty: the only J-Link in /dev/serial/by-id
        double sigma = 0.05;         // noise of one 3D range (m)
        double range_offset = 0.0;   // added to every range (m)
        double fresh_s = 0.3;        // a range older than this is not used for a fix
        double vmax = 4.0;           // fastest plausible horizontal speed (m/s) for the jump gate
        double gate_hold_s = 0.5;    // after this long of gated fixes the gate accepts the new position
        std::string raw_dir;         // CSV of ranges and fixes; empty: none
        bool require_survey = false; // no fixes until the tag's survey gives a layout (anchors move between sessions)
        double dz_max = 3.0;         // largest plausible anchor-plane-to-tag distance; beyond it the map is wrong
    };

    TagTracker(AnchorMap map, Options opt) : map_(std::move(map)), opt_(std::move(opt)) {}
    ~TagTracker() { stop(); }

    void start();
    void stop();

    // Tag height above the floor (m) used until UWB measures its own (the height on the ground at start).
    void set_tag_height(double h) { tag_h_ = h; }
    // True while the vehicle is armed: the anchor layout must not change in flight.
    void set_layout_locked(bool l) { locked_ = l; }

    // Ask the tag to survey again: only the missing pairs (keeps the measured ones), or everything.
    // Never call while armed: a survey pauses ranging for seconds.
    void request_survey_more() { cmd_ = CMD_SURVEY_MORE; }
    void request_resurvey() { cmd_ = CMD_RESURVEY; }

    std::vector<Fix> take();  // fixes since the last call
    TagStats stats() const;

private:
    struct Meas {
        Clock::time_point t;
        double range;  // m
        bool nlos;
    };

    void run();
    void handle(const Packet& pkt, Clock::time_point now);
    void on_range(const Range& r, Clock::time_point now);
    void on_pair(const AnchorPair& p);
    static std::string find_port(std::string& note);
    void open_raw();
    void raw_line(const std::string& s);

    AnchorMap map_;  // only touched by the tracker thread
    Options opt_;
    std::thread th_;
    std::atomic<bool> stop_{false};
    std::atomic<int> cmd_{0};
    Status last_logged_status_{};
    std::atomic<double> tag_h_{0.0};
    std::atomic<bool> locked_{false};

    // tracker thread only
    std::map<int, Meas> latest_;
    std::map<std::pair<int, int>, AnchorPair> pairs_;
    int survey_version_ = -1;
    bool dz_valid_ = false;
    double dz_est_ = 0;
    Clock::time_point dz_time_{};
    bool have_last_ = false;
    Clock::time_point last_t_{};
    double last_x_ = 0, last_y_ = 0;
    bool gating_ = false;
    Clock::time_point gate_since_{};
    std::FILE* raw_ = nullptr;
    Clock::time_point raw_flush_{}, t0_ = Clock::now();

    mutable std::mutex mu_;
    std::vector<Fix> fixes_;
    bool has_status_ = false;
    Status status_;
    std::string raw_name_;

    std::atomic<uint64_t> ok_{0}, fail_{0}, crc_{0}, nfix_{0}, rejected_{0}, gated_{0}, dropped_{0}, nlos_{0};
    std::atomic<int> fresh_{0}, host_pairs_{0};
    std::atomic<bool> map_ready_{false};
    std::atomic<bool> port_open_{false}, using_survey_{false}, h_uwb_{false};
    std::atomic<double> survey_rms_{0.0}, tag_h_uwb_{NAN};
};

}  // namespace uwb
