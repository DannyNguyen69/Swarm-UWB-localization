# UWB → ArduPilot trên Raspberry Pi 4

Pi đọc tag UWB, tính vị trí ngang, gửi cho flight controller chạy **ArduCopter 4.7**, và đẩy toàn bộ telemetry của FC qua UDP tới QGC trên máy NUC.

```
                                        Raspberry Pi 4 (~/dark-landing)
 Tag UWB ──USB──> uwb_bridge.py ──ODOMETRY──> UDP 127.0.0.1:14540 ┐
                  (uwb/)                                          fc_router ──UART5 (ttyAMA5) 921600──> FC SERIAL2
                                              UDP ──> NUC:14550 ──┘ (src/main, C++ cũ của bạn)          ArduCopter 4.7
                                                       QGC                                            EKF3: vị trí ngang từ UWB,
                                                                                                      độ cao baro, yaw la bàn
```

`fc_router` là chương trình cũ của bạn (`src/main/fc_router.cpp`), **giữ nguyên, không sửa**: chỉ một tiến trình được giữ `/dev/ttyAMA5`, nên nó giữ cổng rồi chia ra QGC và chương trình nội bộ. Dòng `fc_gps_hz=0.0` trong log của nó là bình thường (GPS đã tắt).

## Thư mục

Nguồn nằm ở `~/UWB/pi/` trên NUC, `sync.sh` chép sang `~/dark-landing/` trên Pi (chỉ thêm file, không xóa gì của dự án cũ).

```
pi/                          →  ~/dark-landing/ trên Pi
├── README.md                    (file này)
├── sync.sh                      chạy trên NUC: chép mọi thứ sang Pi
├── uwb/
│   ├── uwb_bridge.py            tag → vị trí → ODOMETRY cho FC; đo HEADING; ra lệnh calib anchor
│   ├── lib/                     (tạo bởi sync.sh từ host/) uwbproto, survey_solver, uwb_viewer
│   └── .venv/                   (tạo bởi setup_pi.sh) pymavlink, pyserial, numpy
├── scripts/                     các lệnh chạy, xem bên dưới
├── config/
│   ├── uwb.env                  cấu hình chạy: QGC_IP, HEADING, ORIGIN, TAG_Z... (sửa trên Pi, sync không ghi đè)
│   ├── anchors.toml             (từ host/) tọa độ anchor và độ cao treo
│   ├── ardupilot_uwb_extnav.param   tham số cho FC
│   └── px4_uwb_ev.params        (cũ) tham số PX4
├── systemd/                     dl-router.service, dl-uwb.service: tự chạy khi bật Pi
├── docs/
│   ├── ardupilot.md             tham số, khác biệt PX4/ArduPilot và lý do
│   └── px4.md                   (cũ) hướng dẫn PX4
└── logs/                        (trên Pi) router.log, uwb.log
```
Phần còn lại của `~/dark-landing/` (`src/`, `build/`, `ref/`, `docs/hw.md`) là dự án cũ của bạn, không bị đụng tới.

## Cài đặt (một lần)

```bash
# Trên NUC. SSHPASS để đăng nhập bằng mật khẩu (hoặc dùng khóa SSH rồi bỏ dòng này)
export SSHPASS='<mật khẩu pi4>'
pi/sync.sh pi4@pi4.local
ssh pi4@pi4.local 'cd ~/dark-landing && scripts/setup_pi.sh'      # cần internet trên Pi
```
Sửa `config/anchors.toml` hoặc code ở NUC rồi chạy lại `pi/sync.sh` là đủ.

**FC:** cần các tham số trong `config/ardupilot_uwb_extnav.param` (đã ghi vào FC ngày 2026-11-04). Riêng `VISO_POS_X/Y/Z` (vị trí anten tag so với tâm drone, hệ FRD, z xuống) là của bạn, hiện đang là 0.

## Chuẩn bị trước mỗi lần bay

Sửa trên Pi: `nano ~/dark-landing/config/uwb.env`

| Biến | Ý nghĩa |
|---|---|
| `QGC_IP` | IP của NUC (đang `192.168.0.2`; đổi nếu router cấp IP khác: `ip -4 addr` trên NUC) |
| `ORIGIN` | vĩ độ,kinh độ,độ cao(m) **thật** của chỗ bay, lấy từ điện thoại. Giá trị đang để là **tạm** (khoảng Hà Nội). ArduPilot dùng nó để tính độ lệch từ của la bàn |
| `TAG_Z` | độ cao của tag **so với sàn** tại chỗ drone đang nằm lúc khởi động (m). Drone nằm trên **bàn** thì là chiều cao bàn cộng độ cao tag so với mặt bàn; nằm dưới sàn thì chỉ là độ cao tag so với sàn (~0,1–0,2). Sai thì vị trí ngang sai nhiều, vì nó quyết định dz dùng để đổi khoảng cách 3D sang khoảng cách ngang |
| `HEADING` | **bắt buộc**, để trống thì không chạy: hướng la bàn của cạnh A1→A2 |

**Đo `HEADING`.** Đây là hằng số của căn phòng (cạnh A1→A2 chỉ về hướng nào so với Bắc), không phải yaw hiện tại của drone, nên không thể đọc thẳng từ la bàn lúc bay. Đo nó bằng chính la bàn của FC:
1. Chạy `scripts/run_router.sh` (hoặc `run_all.sh` sau khi đã đặt tạm `HEADING`).
2. Đặt drone **nằm phẳng, đứng yên, mũi hướng từ A1 sang A2** (càng thẳng càng tốt), cách xa kim loại và dây điện.
3. `scripts/measure_heading.sh` rồi chép số in ra vào `HEADING=`. Mũi drone không nhất thiết phải hướng A1→A2: cứ căn theo cạnh nào dễ (ví dụ `scripts/measure_heading.sh --nose 2:1` hoặc `--nose 2:3`), script tự tính góc của cạnh đó từ tọa độ anchor. Căn theo hai cạnh khác nhau rồi so hai kết quả là cách tự kiểm tra tốt.

Sai hướng nhiều thì PosHold vẽ vòng hoặc bay mất kiểm soát. Calib la bàn của FC tại đúng chỗ bay trước.

**Độ cao treo anchor** (`anchor_height` trong `config/anchors.toml`, đã đo: `1.8` m) phải là độ cao thật tính từ sàn: nó đổi khoảng cách 3D sang khoảng cách ngang, sai thì vị trí sai, nhất là khi drone ở thấp. Sửa ở `host/anchors.toml` trên NUC rồi `pi/sync.sh`.

## Lệnh chạy

Trên Pi, trong `~/dark-landing`:

```bash
scripts/run_all.sh        # chạy nền fc_router + uwb_bridge (log trong logs/)
scripts/status.sh         # trạng thái và vài dòng log cuối
tail -f logs/uwb.log      # xem trực tiếp
scripts/stop_all.sh       # dừng cả hai
```
Chạy riêng từng phần khi cần gỡ lỗi: `scripts/run_router.sh`, `scripts/run_uwb.sh`.
Các lệnh khác: `scripts/measure_heading.sh` (đo HEADING), `scripts/resurvey.sh` (bắt tag đo lại khoảng cách giữa anchor, chỉ làm khi drone nằm dưới đất).

Dòng trạng thái của `uwb_bridge` (2 giây một dòng):
```
[12:29:08] gửi 30 Hz | N +3.43 E -2.63 m | đo ok 78/s lỗi 24/s | anchor dùng 4 | tag cao 0.10 m (FC) | bản đồ tự calib | ArduPilot disarmed | origin OK | độ cao: không lấy từ UWB
```
**UWB không đưa độ cao cho EKF:** gói ODOMETRY buộc phải có z hữu hạn nên bridge gửi z = 0, và EKF3 chỉ bỏ qua nó khi `EK3_SRC1_POSZ` là baro (đã kiểm tra trong mã nguồn 4.7). Bridge đọc `EK3_SRC1..3_POSZ` từ FC và **không gửi gì** nếu có tham số nào là ExternalNav (dòng trạng thái ghi `độ cao: ĐANG CHẶN GỬI`). Khoảng cách 3D của tag chỉ được dùng để tính vị trí ngang.

`gửi` nên khoảng 30 Hz, `anchor dùng` từ 3 trở lên (ít hơn thì **không gửi gì**, để FC tự nhận ra mất dữ liệu), `origin OK`, và nếu bản đồ ghi `anchors.toml (tag chưa calib xong…)` thì đang dùng tọa độ trong file (hiện là bản đồ 6 cặp đo ngày 01/11, đã đối chiếu lại ngày 04/11).

**Tự chạy khi bật Pi:**
```bash
sudo cp ~/dark-landing/systemd/dl-*.service /etc/systemd/system/ && sudo systemctl daemon-reload
sudo systemctl enable --now dl-router dl-uwb
journalctl -u dl-router -u dl-uwb -f
```
(`dl-uwb` cần `HEADING` đã đặt, nếu không nó thoát và thử lại mỗi 3 giây.)

## Mở QGC trên NUC

1. Mở QGroundControl. Kết nối UDP mặc định (cổng **14550**) sẽ tự nhận xe, **không cần thêm Comm Link**.
2. Nếu không thấy: kiểm tra `QGC_IP` trong `uwb.env` có đúng IP NUC không, và `scripts/status.sh` báo router đang chạy.
3. Nếu FC vẫn cắm USB vào NUC, QGC có thể có hai liên kết tới cùng một xe. Vẫn dùng được; muốn gọn thì tắt Auto-connect USB trong *Application Settings → General*.

## Danh sách kiểm tra trước khi hover

**Trên bàn, tháo cánh quạt:**
1. QGC báo **`EKF3 IMU0 is using external nav data`** (*Vehicle Messages* hoặc cửa sổ thông báo). Đã thấy dòng này khi chạy thử.
2. Cầm drone đi về phía A2: vị trí trong QGC (*MAVLink Inspector* → `LOCAL_POSITION_NED`: x Bắc, y Đông) phải đổi **đúng hướng la bàn**. Đi ngược hướng nghĩa là `HEADING` sai.
3. Xoay drone tại chỗ: x, y gần như đứng yên. Nếu vẽ vòng thì `VISO_POS_*` hoặc yaw sai.
4. Yaw trên QGC khớp la bàn thật.

**Các lỗi prearm đang có** (thấy khi chạy thử ngày 2026-11-04):
- `PreArm: RC not found`: chưa có điều khiển. Cần tay khiển hoặc joystick trong QGC, hoặc tắt kiểm tra này nếu bạn chủ ý bay không RC.
- `PreArm: OSD_TYPE2 not compatible with first OSD`: `OSD_TYPE = 1`, `OSD_TYPE2 = 5` (MSP DisplayPort). Cổng SERIAL2 từng chạy MSP DisplayPort đã chuyển cho Pi nên `OSD_TYPE2 = 5` không còn dùng được. Đặt `OSD_TYPE2 = 0` nếu không cần OSD thứ hai. **Mình chưa tự đổi.**
- `PreArm: SERIAL2_OPTIONS bit 10 ...`: **đã sửa** (`SERIAL2_OPTIONS = 0`).

**Bay thử:** buộc dây an toàn; cất cánh ở **AltHold** ở 0,5–1 m; chuyển sang **PosHold** (công tắc chế độ 6 đã là PosHold, 4 là AltHold); drone trôi hoặc vẽ vòng thì **chuyển ngay về AltHold**. Sau chuyến bay xem log `.bin`: `VISP` (cột `Ignored` phải bằng 0) và innovation của ExternalNav, chỉnh `VISO_DELAY_MS`.

## Đã thử và chưa thử

**Đã thử (2026-11-04):** FC → Pi trên `ttyAMA5` ở 921600; `fc_router` chạy với ArduPilot và đẩy tới NUC; `uwb_bridge` với tag thật gửi ODOMETRY qua router, FC báo `EKF3 IMU0 is using external nav data`, `origin set`; một QGC giả trên NUC nhận ODOMETRY, vị trí và STATUSTEXT; `measure_heading.sh`, `resurvey.sh`, `run_all.sh / status.sh / stop_all.sh`.

**Chưa thử:** QGC thật; arm; bay; `systemd`; `HEADING` thật (lúc thử dùng `0`); `VISO_POS_*`; độ lệch cố định khoảng 20 cm của tag (xem `PLAN.md`).

**Còn hạn chế:**
- Tag trên Pi chỉ có 4/6 cặp anchor (thiếu A2–A3, A2–A4 vì A2 yếu, khoảng 46–60% lượt đo thành công) nên chưa tự giải được bản đồ; bridge dùng tọa độ trong `anchors.toml`.
- Mỗi lượt đo bị mất khoảng 20–25% (A2, A4 yếu), nên đôi lúc chỉ có 3 anchor.
- Baro trôi khi FC mới bật nguồn; bridge lấy độ cao tương đối so với lúc còn nằm dưới đất để không bị lệch.

## Tự chạy khi Pi khởi động (C++)

`cpp/` là bản C++ của uwb_bridge (MAVSDK, cùng giao thức và cùng hình học với bản Python). Systemd chạy `dl-router` rồi `dl-uwb`.

```bash
# trên Pi, một lần (sau khi sync.sh từ NUC và đặt HEADING trong config/uwb.env)
scripts/build_uwb.sh           # biên dịch cpp/build/uwb_bridge
scripts/install_services.sh    # cài và bật dl-router + dl-uwb
journalctl -u dl-uwb -f        # xem log
```

`run_uwb.sh` tự dùng binary C++ nếu có (`USE_PY=1` để ép dùng Python). Thử không nối FC: `cpp/build/uwb_bridge --heading 196 --dry-run`.

Tần số: gửi `VISION_POSITION_ESTIMATE` đúng `RATE` Hz (mặc định 30, tối đa 50) theo lịch tuyệt đối, mỗi gói đóng dấu thời gian trung bình của các mẫu trong cửa sổ. Hai gói cách nhau dưới 21 ms thì gộp (EKF3 bỏ mẫu cách dưới 20 ms). Không gửi gói cũ hơn 150 ms, không gửi khi dưới 3 anchor, hoặc khi POSZ của EKF còn là ExternalNav. Cứ 2 giây log một dòng: Hz thực, khoảng cách gói, số lần gộp/trống/trễ. Đo thực tế trên Pi: 30.0 Hz, khoảng cách 33.3 ms, jitter < 0.1 ms.

## Bộ lọc UWB trong bridge C++ (từ 2026-11-04, sau log bay 6-8)

- **Độ cao tag lấy từ UWB**, không lấy từ baro: với ≥ 4 anchor, bridge giải 3D (x, y, |dz|) rồi làm mượt |dz|. Baro bị gió cánh quạt đẩy lên 5–9 m khi bay, làm vị trí sai vài mét. Độ cao FC chỉ in ra để so (`tag cao UWB ... | FC ...`). `TAG_Z` chỉ dùng tới khi UWB đo được.
- **Loại ngoại lai**: 4 anchor mà sai số dư > 15 cm thì thử bỏ từng anchor; vẫn xấu thì bỏ fix. Fix nhảy nhanh hơn `--vmax` (4 m/s) bị chặn tối đa 0.5 s.
- **Covariance thật** gửi cho EKF: theo hình học, độ khuếch đại r/h khi tag gần ngay dưới anchor, cờ NLOS của firmware và sai số dư (`σ gửi` trong log).
- **Ghi khoảng cách thô**: `logs/uwb_raw/uwb_raw_*.csv` (giữ 30 file mới nhất). Journal của Pi được lưu qua reboot (`/etc/systemd/journald.conf.d/50-persistent.conf`).
- **Bản đồ anchor mỗi lần bật nguồn** (anchor được treo lại mỗi buổi bay): `REQUIRE_SURVEY=1` (mặc định) chỉ dùng bản đồ do tag tự calib trong lần bật nguồn này, không bao giờ dùng `anchors.toml`. Chưa có bản đồ thì không gửi vị trí, FC báo `VisOdom: not healthy` và không cho arm. Log ghi `bản đồ CHƯA CÓ, đang calib (n/6 cặp)`.
- Firmware tag v0.7: sau khi bật nguồn survey tối đa 2 lượt; nếu vẫn thiếu cặp, bridge gửi lệnh `CMD_SURVEY_MORE` (0x81) mỗi 10 s **chỉ khi chưa arm**, tag đo tiếp các cặp còn thiếu và giữ các cặp đã đo. Không bao giờ survey khi đang bay.
- Khi chưa arm mà hơn một nửa fix bị loại trong 4 s, bridge coi bản đồ không khớp và chặn gửi; kéo dài 15 s thì yêu cầu tag đo lại toàn bộ. Độ cao ước lượng bị giới hạn `--dz-max` (3 m).
- File thô có thêm dòng `P` (từng cặp anchor đo được) và `S` (trạng thái survey của tag).

Bản Python (`USE_PY=1`) chưa có các bộ lọc này.
