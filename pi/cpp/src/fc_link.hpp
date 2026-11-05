// MAVLink link to an ArduPilot flight controller through fc_router's local UDP port (MAVSDK MavlinkDirect).
//
// What it does for the vision-position stream:
//   - sends VISION_POSITION_ESTIMATE (no velocity fields, so no NaN is needed and EKF3 never sees an
//     external velocity); z is the constant 0, harmless only while no EKF3 source set takes height from
//     ExternalNav, which is checked: nothing is sent unless EK3_SRC1..3_POSZ are all != 6
//   - makes sure ATTITUDE and LOCAL_POSITION_NED arrive (only asks when they are missing, so QGC's own
//     rates on the shared link are not overridden)
//   - sets the EKF origin when the FC has none (needed without GPS)
#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace mavsdk {
class Mavsdk;
class MavlinkDirect;
}  // namespace mavsdk

namespace uwb {

class FcLink {
public:
    struct Config {
        std::string url = "udpout://127.0.0.1:14540";
        uint32_t sysid = 1;
        uint8_t compid = 197;  // MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
        bool have_origin = false;
        double lat = 0, lon = 0, alt = 0;  // EKF origin, degrees and metres
    };
    enum class ZState { Unknown = 0, Ok = 1, Blocked = 2 };

    FcLink();
    ~FcLink();

    // Connects and waits for the autopilot. Returns false if stop was requested or the link cannot be opened.
    bool start(const Config& cfg, const std::atomic<bool>& stop);
    void maintain();  // call about once a second

    bool connected() const { return connected_; }
    bool armed() const { return armed_; }
    bool origin_set() const { return origin_set_; }
    ZState z_state() const { return static_cast<ZState>(z_state_.load()); }
    bool attitude_ok() const { return att_seen_; }
    bool ready() const { return connected() && z_state() == ZState::Ok && attitude_ok(); }
    std::string z_summary() const;  // "EK3_SRC1..3_POSZ = 1,1,1"
    // Height above the EKF origin in metres, if LOCAL_POSITION_NED is fresh.
    std::optional<double> height() const;

    // VISION_POSITION_ESTIMATE: north/east in metres from the EKF origin, variances in m^2.
    bool send_vision_position(uint64_t usec, double north, double east, double var_n, double var_e);
    uint64_t sent() const { return sent_; }
    uint64_t send_failed() const { return send_failed_; }

private:
    bool send(const std::string& name, const std::string& json, bool targeted);
    void on_message(const std::string& name, uint32_t sys, uint32_t comp, const std::string& json);
    void eval_z();
    std::string z_summary_locked() const;
    void request_interval(int msg_id, double hz);

    std::unique_ptr<mavsdk::Mavsdk> mavsdk_;
    std::unique_ptr<mavsdk::MavlinkDirect> direct_;
    Config cfg_;
    uint32_t fc_sysid_ = 0;

    std::atomic<bool> connected_{false}, armed_{false}, origin_set_{false}, att_seen_{false};
    std::atomic<int> z_state_{0};
    std::atomic<double> roll_{0}, pitch_{0}, yaw_{0}, z_ned_{0};
    std::atomic<int64_t> att_ns_{0}, pos_ns_{0}, connected_ns_{0};
    std::atomic<uint64_t> sent_{0}, send_failed_{0};
    int64_t last_param_poll_ns_ = 0, last_req_att_ns_ = 0, last_req_pos_ns_ = 0;

    mutable std::mutex mu_;
    std::map<std::string, double> params_;
};

}  // namespace uwb
