# Đưa vị trí UWB từ Raspberry Pi xuống ArduPilot Copter 4.7

Tài liệu kỹ thuật về phần ArduPilot: tham số, khác biệt với PX4 và lý do. **Cách cài và chạy nằm ở [../README.md](../README.md).** Phần PX4 cũ ở [px4.md](px4.md).

```
 Tag UWB ──USB──> Raspberry Pi 4                                          Pixhawk, ArduCopter 4.7
                  uwb_bridge.py ──ODOMETRY──> fc_router ──UART5 921600──> SERIAL2 (EKF3: ExternalNav)
                                                  └──UDP 14550──> QGC (máy NUC)
```

> **Trạng thái (2026-11-04):** các tham số dưới đây đã ghi vào FC thật (Saolah743, ArduCopter 4.7.0). Đã chạy thử đường Pi → router → FC với tag thật: FC báo `EKF3 IMU0 is using external nav data` và `origin set`, QGC giả trên NUC nhận được ODOMETRY, vị trí và STATUSTEXT. **Chưa arm, chưa bay, chưa thử bằng QGC thật.**

## Cấu hình đang áp dụng: PosHold với UWB chỉ cho vị trí ngang, độ cao từ baro

- Vị trí ngang (x, y): **ExternalNav** (UWB). Vận tốc ngang và dọc: **không dùng** (`VELXY = VELZ = 0`). Độ cao: **baro**. Yaw: la bàn. **GPS đã tắt** (`GPS1_TYPE`: 1 → 0); muốn bay ngoài trời thì đặt lại 1 và `EK3_SRC1_*` về 3.
- Đã kiểm tra trong `AP_NavEKF_Source::pre_arm_check`: cấu hình này hợp lệ, chỉ đòi có baro và `VISO_TYPE` ≠ 0. Mặc định cũ `VELXY = VELZ = 3` (GPS) mà tắt GPS thì sẽ bị chặn arm với "EK3 sources require GPS".
- `ODOMETRY` của script có vận tốc là NaN: EKF3 tự loại mẫu có NaN (`writeExtNavVelData`) nên không ảnh hưởng.
- Công tắc chế độ bay 6 (`FLTMODE6 = 16`) đã là **PosHold**, công tắc 4 là AltHold.
- Chừng nào Pi chưa gửi gói thì prearm báo `VisOdom: not healthy`, đó là bình thường.

## Khác biệt với PX4 (đọc từ mã nguồn ArduPilot 4.7)

| | PX4 1.18 | **ArduPilot 4.7** |
|---|---|---|
| Hệ tọa độ của `ODOMETRY` | `MAV_FRAME_LOCAL_NED` | **Chỉ nhận `MAV_FRAME_LOCAL_FRD`**, khác đi thì bị bỏ im lặng (`GCS_MAVLINK::handle_odometry`). Với `VISO_TYPE=1`, vị trí được dùng như NED, không xoay |
| Độ bất định | Phương sai riêng từng trục | **Một con số chung** `√(cov[0]+cov[6]+cov[11])`, kẹp tối thiểu bằng `VISO_POS_M_NSE`. Script đặt phương sai z = 0 để không làm phồng con số này |
| Tư thế (quaternion) | Không cần | Phải hợp lệ. Script **gửi lại tư thế của chính FC** (lấy từ `ATTITUDE`). Vì `EK3_SRC1_YAW = 1` (la bàn), yaw trong gói không được dùng |
| EKF origin | Không cần | **Bắt buộc** khi không có GPS. Script hỏi FC (`MAV_CMD_REQUEST_MESSAGE`) xem đã có origin chưa; sau 3 s chưa có thì gửi `SET_GPS_GLOBAL_ORIGIN` mỗi giây cho tới khi FC báo `GPS_GLOBAL_ORIGIN` |
| Timestamp | TIMESYNC | ArduPilot tự bù jitter bằng `correct_offboard_timestamp_usec_to_ms`, cộng thêm `VISO_DELAY_MS` |
| Tốc độ gửi các message | PX4 tự gửi | Cổng companion có thể không gửi gì. Script yêu cầu `ATTITUDE` 20 Hz và `LOCAL_POSITION_NED` 10 Hz bằng `MAV_CMD_SET_MESSAGE_INTERVAL`, **chỉ khi message đó không đến** trong 3 s, để không đè lên tốc độ QGC đặt trên cùng đường truyền |

## Tham số

Mission Planner → *Config* → *Full Parameter List* → *Load from file* → `config/ardupilot_uwb_extnav.param` → *Write Params* → **reboot** (`VISO_TYPE` và `SERIAL2_*` chỉ có hiệu lực sau khi khởi động lại).

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| `SERIAL2_PROTOCOL` | 2 | MAVLink 2 trên TELEM2 |
| `SERIAL2_BAUD` | 921 | 921600 baud |
| `SERIAL2_OPTIONS` | **0** | **Không được đặt bit 10** (1024): ArduPilot 4.7 chặn arm với `SERIAL2_OPTIONS bit 10 ('no forward mavlink') must not be set; use MAVn_OPTIONS bit 1 instead`. Lỗi này do mình cấu hình sai lúc đầu (theo tài liệu T265 cũ), đã sửa trên FC ngày 2026-11-04 |
| `GPS1_TYPE` | 0 | Tắt GPS (từ 4.6 tên là `GPS1_TYPE`, không còn `GPS_TYPE`) |
| `VISO_TYPE` | 1 | Visual odometry qua MAVLink |
| `VISO_DELAY_MS` | 10 | Mặc định. Script gắn thời điểm đo thật nên trễ còn lại nhỏ; chỉnh theo log |
| `VISO_POS_M_NSE` | 0,1 m | Mức nhiễu tối thiểu (khoảng hợp lệ 0,1–10). Mặc định là 0,2 |
| **`VISO_POS_X/Y/Z`** | **tự đo** | Vị trí anten tag so với tâm drone (FRD: x trước, y phải, **z xuống**) |
| `EK3_SRC1_POSXY` | 6 | Vị trí ngang từ ExternalNav |
| `EK3_SRC1_VELXY` | 0 | Không có vận tốc từ UWB |
| `EK3_SRC1_POSZ` | 1 | **Độ cao từ baro** (UWB chỉ cho vị trí x, y) |
| `EK3_SRC1_VELZ` | 0 | |
| `EK3_SRC1_YAW` | 1 | Yaw từ la bàn |

Kiểm tra thêm:
- `FS_EKF_ACTION`: hành động khi EKF xấu (mặc định 1 = Land; 2 = AltHold).
- Chế độ bay: **Loiter** hoặc **PosHold** dùng vị trí UWB; **AltHold** là chế độ dự phòng khi UWB có vấn đề. Gán AltHold vào một công tắc dễ với tới.

## Chạy, kiểm tra trên bàn và bay thử

Xem [../README.md](../README.md): cài đặt, đo `HEADING`, lệnh chạy, cách mở QGC và danh sách kiểm tra trước khi hover.

Ý chính về hai tham số hay nhầm:
- **`ORIGIN`** (vĩ độ, kinh độ, độ cao): ArduPilot dùng nó để tự tính độ lệch từ của la bàn (`COMPASS_AUTODEC`), nên phải là tọa độ **thật** của chỗ bay.
- **`HEADING`**: hướng la bàn của cạnh A1→A2. Đây là hằng số của căn phòng, không phải yaw hiện tại của drone, nên không thể lấy trực tiếp từ la bàn lúc bay. Script `measure_heading.sh` đo nó bằng chính la bàn của FC khi drone nằm yên với mũi hướng từ A1 sang A2.

## Nguồn

- Mã nguồn ArduPilot tag `Copter-4.7.0`: `libraries/GCS_MAVLink/GCS_Common.cpp` (`handle_odometry`, `handle_set_gps_global_origin`), `libraries/AP_VisualOdom/` (tham số `VISO_*`, `AP_VisualOdom_MAV.cpp`), `libraries/AP_NavEKF/AP_NavEKF_Source.cpp` (`EK3_SRC1_*`), `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp` (`writeExtNavData`, bỏ mẫu NaN và mẫu dưới 20 ms)
- [Copter-4.7.0 release](https://github.com/ArduPilot/ardupilot/releases/tag/Copter-4.7.0)
- [ArduPilot non-GPS navigation](https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html), [VIO tracking camera (tham số EK3_SRC, SERIAL2, EKF origin)](https://ardupilot.org/copter/docs/common-vio-tracking-camera.html)
- Diễn đàn ArduPilot: [VisOdom not healthy / external nav](https://discuss.ardupilot.org/t/vision-position-estimate-not-being-able-to-inspect-in-mission-planner/111582), [VISO_TYPE cần reboot](https://discuss.ardupilot.org/t/about-gps-non-gps-transitions/71934)
