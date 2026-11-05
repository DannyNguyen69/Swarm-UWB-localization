> **Tài liệu PX4 (cũ).** Viết khi thư mục còn tên `deploy/`; nay là `pi/`, và `uwb_to_px4.py` là `uwb/uwb_bridge.py --fw px4`. Bridge mặc định nối qua `fc_router` (`--fc udpout:127.0.0.1:14540`); các ví dụ dưới đây nối thẳng cổng serial (`--fc /dev/ttyAMA5`) và chỉ dùng khi không chạy router. Dự án hiện chạy **ArduPilot**: xem [../README.md](../README.md). Phần mục 1 (rsync) và systemd bên dưới đã được thay bằng `sync.sh` và `systemd/dl-*.service`.

# Đưa vị trí UWB từ Raspberry Pi xuống PX4

```
 Tag UWB ──USB──> Raspberry Pi 4 ──UART 921600──> Pixhawk 6C (TELEM2)
                  uwb/uwb_bridge.py --fw px4              EKF2: external vision (vị trí ngang)
                  giải vị trí 2D, xoay sang NED,     độ cao: rangefinder
                  gửi MAVLink ODOMETRY 50 Hz         hướng (yaw): la bàn
```

ArduPilot 4.7: xem [README_ardupilot.md](README_ardupilot.md).

Đã thử với PX4 SITL 1.18 và tag thật: EKF2 fuse EV (`cs_ev_pos = True`, `xy_valid = True`), innovation khoảng 2 cm, không mẫu nào bị loại. **Chưa thử trên Pixhawk thật.**

---

## 1. Chép code sang Pi

Pi chỉ cần `host/` (bộ giải mã gói, bộ giải tọa độ, `anchors.toml`) và `deploy/`. Từ máy này:

```bash
rsync -av --exclude '.venv' --exclude '__pycache__' \
      ~/UWB/host ~/UWB/deploy  pi@<ip-cua-pi>:~/UWB/
```

## 2. Cài đặt trên Pi (một lần)

```bash
cd ~/UWB
bash scripts/setup_pi.sh
```

Script cài `python3-serial`, `numpy` và `pymavlink` (trong `uwb/.venv`), đồng thời thêm user vào nhóm `dialout`.

**Bật UART trên GPIO14/15** để nối TELEM2:

1. `sudo raspi-config` → *Interface Options* → *Serial Port*: login shell qua serial → **No**, bật phần cứng serial → **Yes**.
2. Thêm vào cuối `/boot/firmware/config.txt`:
   ```
   enable_uart=1
   dtoverlay=disable-bt
   ```
   `disable-bt` đưa UART PL011 chuẩn ra GPIO14/15, thành `/dev/ttyAMA0`. Nếu không có dòng này, Pi dùng mini-UART, tốc độ phụ thuộc xung CPU và không ổn định ở 921600.
3. `sudo systemctl disable hciuart`, rồi `sudo reboot`.

## 3. Nối dây

| Raspberry Pi 4 | Pixhawk 6C **TELEM2** (JST-GH 6 chân) |
|---|---|
| Chân 8 — GPIO14 **TXD** | Chân 3 — **RX** |
| Chân 10 — GPIO15 **RXD** | Chân 2 — **TX** |
| Chân 6 — **GND** | Chân 6 — **GND** |
| *(không nối)* | Chân 1 — 5V |

- Mức logic 3,3 V ở cả hai phía, nối thẳng được.
- **Không cấp nguồn Pi từ TELEM2.** Dùng BEC 5 V riêng, ít nhất 3 A.
- Tag cắm vào cổng USB của Pi. Pi tự nhận tag (đường dẫn `/dev/serial/by-id/usb-SEGGER_J-Link_…`).
- Thử nhanh trên bàn có thể cắm USB từ Pi sang cổng USB của Pixhawk (`--fc /dev/ttyACM0`), không cần UART. Khi bay thì nên dùng TELEM2.

## 4. Tham số PX4

Trong QGroundControl: *Parameters* → *Tools* → *Load from file* → chọn `config/px4_uwb_ev.params`, rồi **reboot** Pixhawk.

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| `MAV_1_CONFIG` | 102 (TELEM 2) | Bật MAVLink trên TELEM2. Nếu `MAV_1` đang dùng cho việc khác thì dùng `MAV_2_*` |
| `MAV_1_MODE` | 2 (Onboard) | Chế độ cho máy tính đồng hành |
| `SER_TEL2_BAUD` | 921600 | Khớp với `--fc-baud` |
| `EKF2_GPS_CTRL` | 0 | Không dùng GPS (bay trong nhà) |
| `EKF2_EV_CTRL` | 1 | **Chỉ fuse vị trí ngang** từ EV, không fuse độ cao hay yaw |
| `EKF2_EVP_NOISE` | 0,1 m | Đã kiểm chứng bằng mô phỏng (`ekf/px4_ev_sim.py`) |
| `EKF2_EVP_GATE` | 5 | Mặc định |
| `EKF2_EV_DELAY` | 5 ms | Điểm xuất phát; chỉnh theo log (xem mục 8) |
| `EKF2_HGT_REF` | 2 (range) | Độ cao lấy từ rangefinder |
| `EKF2_RNG_CTRL` | 2 | Luôn dùng rangefinder |
| **`EKF2_EV_POS_X/Y/Z`** | **tự đo** | Vị trí anten tag so với tâm drone, hệ FRD (x trước, y phải, **z xuống**). Tag cao hơn tâm 10 cm thì `EKF2_EV_POS_Z = -0.10` |

Ngoài ra, kiểm tra **failsafe khi mất vị trí** ở chế độ Position. Tham số cụ thể khác nhau tùy bản PX4; tìm "position loss" hoặc "navigation loss" trong QGC. Nên để chuyển sang Altitude mode hoặc hạ cánh, và luôn có công tắc chuyển sang Altitude mode trên tay khiển.

## 5. Ba thông số chạy cần đo một lần

**`--heading`**: hướng la bàn của cạnh **A1 → A2**, tính bằng độ (0 = Bắc, 90 = Đông, theo chiều kim đồng hồ). Đứng ở A1, quay mặt về A2, đọc la bàn điện thoại; đứng xa sắt thép và dây điện. Script xoay hệ UWB (A1 là gốc, x hướng A2, y hướng A4) sang NED theo góc này. Sai góc này thì drone hiểu sai hướng chuyển động, giống lỗi yaw.

**`--tag-z`**: tag cao hơn tâm drone bao nhiêu mét (số dương). Script cộng giá trị này với độ cao PX4 báo về để biết tag đang ở độ cao nào, rồi đổi khoảng cách 3D sang khoảng cách ngang. Khi PX4 chưa báo độ cao, script dùng `tag_height` trong `host/anchors.toml`.

**`host/anchors.toml`** trên Pi: `anchor_height` (độ cao treo anchor) phải đúng. Để `source = "auto"` thì tọa độ anchor lấy từ lần tự calib của tag. **Bản đồ chỉ được thay đổi khi drone disarm**; khi đã arm, script khóa bản đồ.

## 6. Chạy

```bash
cd ~/UWB
uwb/.venv/bin/python uwb/uwb_bridge.py --fw px4 --fc /dev/ttyAMA0 --heading 87 --tag-z 0.10
```

Mỗi 2 giây script in một dòng trạng thái:
```
[19:46:31] gửi 50 Hz | đo ok 92/s lỗi 8/s | anchor dùng 4 | tag cao 0.10 m (FC) | bản đồ tự calib | PX4 disarmed | tự calib tag: xong
```
- `gửi` khoảng 50 Hz, `anchor dùng` từ 3 trở lên. Ít hơn 3 anchor thì **không gửi gì**, để PX4 tự nhận ra mất dữ liệu.
- `PX4 disarmed` hoặc `ARMED`: đã nhận heartbeat. Nếu báo `chưa kết nối`, kiểm tra dây TX/RX (thường bị đảo), tốc độ baud, `MAV_1_CONFIG`.

**Tùy chọn:**
- `--rate 50`: tần số gửi. Có thể đặt 100 ở 921600 baud (xem `ekf/px4_ev_sim.py --rates`).
- `--range-offset 0.21`: cộng 21 cm vào mọi khoảng cách của tag. Lần đo tĩnh ngày 2026-11-01 cho thấy tag đo ngắn hơn khoảng 21 cm (độ trễ anten của tag chưa calib). **Kiểm tra bằng thước trước khi dùng.** Bỏ trống thì drone vẫn hover ổn, chỉ đứng lệch khoảng 20 cm.

**Tự chạy khi bật Pi:** sửa `User`, đường dẫn và tham số trong `deploy/uwb-px4.service`, rồi:
```bash
sudo cp deploy/uwb-px4.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now uwb-px4
journalctl -u uwb-px4 -f
```

## 7. Kiểm tra trên bàn (tháo cánh)

1. QGC → *Analyze Tools* → *MAVLink Inspector*: thấy `ODOMETRY` khoảng 50 Hz.
2. Bảng vị trí local của QGC: x/y có giá trị và ổn định khi drone đứng yên.
3. Cầm drone **đi về phía A2**: vị trí phải đổi theo đúng hướng la bàn đã đo. Đi ngược hướng thì sai `--heading` hoặc yaw.
4. **Xoay drone tại chỗ:** vị trí x/y gần như không đổi. Nếu x/y vẽ vòng thì `EKF2_EV_POS_*` hoặc yaw đang sai.
5. Kiểm tra yaw trong QGC khớp với la bàn thật. La bàn trong nhà là rủi ro lớn nhất; calib la bàn ngay tại chỗ.

## 8. Bay thử lần đầu

1. **Buộc dây an toàn.** Cất cánh ở Altitude mode, độ cao 0,5–1 m.
2. Chuyển sang **Position mode**. Nếu drone trôi hoặc vẽ vòng thì chuyển ngay về Altitude mode.
3. Sau chuyến bay, xem log ULog: topic `estimator_aid_src_ev_pos`. Innovation nên nhỏ (vài cm) và dao động quanh 0, `test_ratio` nhỏ hơn 1.
4. Chỉnh **`EKF2_EV_DELAY`** sao cho innovation nhỏ nhất khi lắc drone qua lại. Script đã gắn thời điểm đo thật và trả lời TIMESYNC, nên phần trễ còn lại chỉ là đường tag → Pi (vài ms).

## Giới hạn đã biết

- UWB **không đo độ cao** (4 anchor cùng độ cao) và **không đo hướng**. Độ cao do rangefinder lo, hướng do la bàn lo.
- Mô phỏng real-to-sim (`sim/`) cho giữ vị trí RMS 2,4 cm, nhưng không có gió, nhiễu la bàn hay thân drone che tag.
- Anchor phải có nguồn ổn định. Một anchor khởi động lại khi đang bay sẽ làm tag dừng đo 1–2 giây để calib lại; script vẫn giữ bản đồ cũ khi đã arm, nhưng sẽ có khoảng trống dữ liệu.
