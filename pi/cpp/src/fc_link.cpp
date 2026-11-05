#include "fc_link.hpp"

#include <mavsdk/mavsdk.hpp>
#include <mavsdk/plugins/mavlink_direct/mavlink_direct.hpp>

#include <cmath>
#include <format>
#include <print>
#include <thread>

namespace uwb {
namespace {

constexpr int kMsgAttitude = 30, kMsgLocalPositionNed = 32, kMsgGpsGlobalOrigin = 49;
constexpr int kCmdRequestMessage = 512, kCmdSetMessageInterval = 511;
constexpr const char* kZParams[] = {"EK3_SRC1_POSZ", "EK3_SRC2_POSZ", "EK3_SRC3_POSZ"};
constexpr int kExternalNav = 6;  // value of EK3_SRCn_POSZ

int64_t now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

double json_num(const std::string& json, const std::string& key)
{
    auto pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return NAN;
    pos = json.find(':', pos);
    if (pos == std::string::npos) return NAN;
    try {
        return std::stod(json.substr(pos + 1));
    } catch (...) {
        return NAN;
    }
}

std::string json_str(const std::string& json, const std::string& key)
{
    auto pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    pos = json.find(':', pos);
    if (pos == std::string::npos) return "";
    pos = json.find('"', pos);
    if (pos == std::string::npos) return "";
    std::string out;
    for (++pos; pos < json.size() && json[pos] != '"'; ++pos) {
        if (json[pos] == '\\' && pos + 1 < json.size()) {
            ++pos;
            out += json[pos] == 'n' ? ' ' : json[pos];
        } else {
            out += json[pos];
        }
    }
    return out;
}

std::string command_long(uint32_t sys, int command, double p1, double p2)
{
    return std::format(
        R"({{"target_system":{},"target_component":1,"command":{},"confirmation":0,)"
        R"("param1":{:.0f},"param2":{:.0f},"param3":0,"param4":0,"param5":0,"param6":0,"param7":0}})",
        sys, command, p1, p2);
}

}  // namespace

FcLink::FcLink() = default;
FcLink::~FcLink() = default;

bool FcLink::start(const Config& cfg, const std::atomic<bool>& stop)
{
    cfg_ = cfg;
    mavsdk::Mavsdk::Configuration mc{cfg.sysid, cfg.compid, true};
    mavsdk_ = std::make_unique<mavsdk::Mavsdk>(mc);
    if (mavsdk_->add_any_connection(cfg.url) != mavsdk::ConnectionResult::Success) {
        std::println(stderr, "[fc] không mở được {}", cfg.url);
        return false;
    }
    std::println("[fc] link {}", cfg.url);

    std::optional<std::shared_ptr<mavsdk::System>> system;
    int waited = 0;
    while (!stop && !(system = mavsdk_->first_autopilot(2.0))) {
        if (waited++ % 5 == 0) std::println("[fc] chờ autopilot qua {} (fc_router đã chạy và FC đã cấp nguồn chưa?)", cfg.url);
    }
    if (!system) return false;
    fc_sysid_ = (*system)->get_system_id();
    std::println("[fc] autopilot sysid {}", fc_sysid_);

    direct_ = std::make_unique<mavsdk::MavlinkDirect>(*system);
    for (const char* name : {"HEARTBEAT", "ATTITUDE", "LOCAL_POSITION_NED", "GPS_GLOBAL_ORIGIN", "PARAM_VALUE", "STATUSTEXT"}) {
        direct_->subscribe_message(name, [this, name](mavsdk::MavlinkDirect::MavlinkMessage m) {
            on_message(name, m.system_id, m.component_id, m.fields_json);
        });
    }
    return true;
}

void FcLink::on_message(const std::string& name, uint32_t sys, uint32_t comp, const std::string& json)
{
    if (sys != fc_sysid_ || comp != 1) return;  // QGC, the router and other components share this link
    const int64_t now = now_ns();
    if (name == "HEARTBEAT") {
        if (json_num(json, "autopilot") != 3) return;  // MAV_AUTOPILOT_ARDUPILOTMEGA
        if (!connected_) {
            connected_ns_ = now;
            connected_ = true;
            std::println("[fc] heartbeat ArduPilot");
        }
        armed_ = (static_cast<int>(json_num(json, "base_mode")) & 128) != 0;
    } else if (name == "ATTITUDE") {
        roll_ = json_num(json, "roll");
        pitch_ = json_num(json, "pitch");
        yaw_ = json_num(json, "yaw");
        att_ns_ = now;
        att_seen_ = true;
    } else if (name == "LOCAL_POSITION_NED") {
        z_ned_ = json_num(json, "z");
        pos_ns_ = now;
    } else if (name == "GPS_GLOBAL_ORIGIN") {
        if (!origin_set_) std::println("[fc] EKF origin: {:.6f}, {:.6f}", json_num(json, "latitude") / 1e7, json_num(json, "longitude") / 1e7);
        origin_set_ = true;
    } else if (name == "PARAM_VALUE") {
        const std::string id = json_str(json, "param_id");
        for (const char* z : kZParams) {
            if (id == z) {
                {
                    std::lock_guard lk(mu_);
                    params_[id] = json_num(json, "param_value");
                }
                eval_z();
            }
        }
    } else if (name == "STATUSTEXT") {
        std::println("[ardupilot] {}", json_str(json, "text"));
    }
}

void FcLink::eval_z()
{
    std::lock_guard lk(mu_);
    for (const char* z : kZParams)
        if (!params_.count(z)) return;
    bool blocked = false;
    for (const char* z : kZParams) blocked |= static_cast<int>(params_[z]) == kExternalNav;
    const int next = blocked ? static_cast<int>(ZState::Blocked) : static_cast<int>(ZState::Ok);
    if (z_state_.exchange(next) != next) {
        if (blocked) std::println("[fc] ⚠ có EK3_SRCn_POSZ = ExternalNav: EKF3 sẽ lấy độ cao từ gói vị trí (luôn 0). KHÔNG GỬI. Đặt về 1 (baro).");
        else std::println("[fc] độ cao không lấy từ UWB ({})", z_summary_locked());
    }
}

std::string FcLink::z_summary() const
{
    std::lock_guard lk(mu_);
    return z_summary_locked();
}

std::string FcLink::z_summary_locked() const
{
    std::string s = "EK3_SRC1..3_POSZ = ";
    bool first = true;
    for (const char* z : kZParams) {
        const auto it = params_.find(z);
        s += (first ? "" : ",") + (it == params_.end() ? std::string("?") : std::to_string(static_cast<int>(it->second)));
        first = false;
    }
    return s;
}

std::optional<double> FcLink::height() const
{
    if (now_ns() - pos_ns_.load() > 2'000'000'000LL || pos_ns_.load() == 0) return std::nullopt;
    return -z_ned_.load();
}

bool FcLink::send(const std::string& name, const std::string& json, bool targeted)
{
    if (!direct_) return false;
    mavsdk::MavlinkDirect::MavlinkMessage msg;
    msg.message_name = name;
    msg.fields_json = json;
    if (targeted) {
        msg.target_system_id = fc_sysid_;
        msg.target_component_id = 1;
    }
    return direct_->send_message(msg) == mavsdk::MavlinkDirect::Result::Success;
}

void FcLink::request_interval(int msg_id, double hz)
{
    send("COMMAND_LONG", command_long(fc_sysid_, kCmdSetMessageInterval, msg_id, 1e6 / hz), true);
}

void FcLink::maintain()
{
    if (!connected_) return;
    const int64_t now = now_ns();

    // Streams: ask only for what is not arriving.
    // At most every 15 s: LOCAL_POSITION_NED does not exist while the EKF has no position, and every request
    // makes MAVSDK log a warning about an ack it does not know.
    if (now - att_ns_.load() > 3'000'000'000LL && now - last_req_att_ns_ > 15'000'000'000LL) {
        last_req_att_ns_ = now;
        request_interval(kMsgAttitude, 20);
    }
    if (now - pos_ns_.load() > 3'000'000'000LL && now - last_req_pos_ns_ > 15'000'000'000LL) {
        last_req_pos_ns_ = now;
        request_interval(kMsgLocalPositionNed, 10);
    }

    // EKF origin: see whether the FC already has one, else set ours after 3 s.
    if (cfg_.have_origin && !origin_set_) {
        send("COMMAND_LONG", command_long(fc_sysid_, kCmdRequestMessage, kMsgGpsGlobalOrigin, 0), true);
        if (now - connected_ns_.load() > 3'000'000'000LL) {
            send("SET_GPS_GLOBAL_ORIGIN",
                 std::format(R"({{"target_system":{},"latitude":{},"longitude":{},"altitude":{},"time_usec":{}}})", fc_sysid_,
                             static_cast<int64_t>(std::llround(cfg_.lat * 1e7)), static_cast<int64_t>(std::llround(cfg_.lon * 1e7)),
                             static_cast<int64_t>(std::llround(cfg_.alt * 1000.0)), now / 1000),
                 true);
        }
    }

    // Height guard: read the three POSZ parameters until known, then re-read every 10 s so a change made
    // later (QGC, another tool) is noticed even if this link never sees the reply to someone else's write.
    bool all_known;
    {
        std::lock_guard lk(mu_);
        all_known = true;
        for (const char* z : kZParams) all_known &= params_.count(z) > 0;
    }
    if (!all_known || now - last_param_poll_ns_ > 10'000'000'000LL) {
        last_param_poll_ns_ = now;
        for (const char* z : kZParams)
            send("PARAM_REQUEST_READ", std::format(R"({{"target_system":{},"target_component":1,"param_id":"{}","param_index":-1}})", fc_sysid_, z), true);
    }
}

bool FcLink::send_vision_position(uint64_t usec, double north, double east, double var_n, double var_e)
{
    if (!std::isfinite(north) || !std::isfinite(east) || !std::isfinite(var_n) || !std::isfinite(var_e)) return false;
    // covariance is the upper triangle of a 6x6 matrix, row-major: x=[0] y=[6] z=[11] roll=[15] pitch=[18] yaw=[20].
    // ArduPilot reads sqrt(c0 + c6 + c11) as one position error and sqrt(c15 + c18 + c20) as the angle error.
    double cov[21] = {};
    cov[0] = var_n;
    cov[6] = var_e;
    cov[11] = 0.0;
    cov[15] = cov[18] = cov[20] = 0.01;
    std::string cs;
    for (int i = 0; i < 21; ++i) cs += std::format("{}{:.8g}", i ? "," : "", cov[i]);
    // roll/pitch/yaw are the FC's own attitude (EK3_SRC1_YAW = compass, so ArduPilot does not use them).
    const std::string json = std::format(
        R"({{"usec":{},"x":{:.4f},"y":{:.4f},"z":0,"roll":{:.5f},"pitch":{:.5f},"yaw":{:.5f},"covariance":[{}],"reset_counter":0}})", usec,
        north, east, roll_.load(), pitch_.load(), yaw_.load(), cs);
    const bool ok = send("VISION_POSITION_ESTIMATE", json, false);
    ok ? ++sent_ : ++send_failed_;
    return ok;
}

}  // namespace uwb
