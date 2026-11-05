# Kế hoạch: Firmware UWB 100 Hz cho DWM1001 (MDEK1001) + tự calib anchor

> Cập nhật: 2026-11-01
> Trạng thái: Firmware v0.1 DS-TWR **chạy được trên phần cứng** (2026-11-01): 1 tag + 4 anchor, 100 lượt đo/giây, chu kỳ 10,00 ms jitter 0.
> Firmware **v0.2** (chưa chạy thử trên phần cứng): dùng calib nhà máy trong OTP (độ trễ anten, công suất phát) và **tự calib vị trí anchor do tag điều phối** (Giai đoạn 5a). v0.2 không tương thích với v0.1 (gói REPORT dài hơn), phải nạp lại cả tag lẫn anchor.
> Các con số đánh dấu *(ước lượng)* là tính toán trên giấy, cần đo thực tế để xác nhận.

---

## 1. Mục tiêu

1. Thay firmware PANS (tối đa 10 Hz) bằng firmware tự viết, đo khoảng cách **DS-TWR** với tổng **~100 lần đo/giây** cho mỗi tag.
2. Tag xuất **khoảng cách thô** (không phải vị trí) qua UART dạng nhị phân, có timestamp, để đưa vào EKF của drone.
3. **4 anchor treo ở 4 góc**, lên nguồn **theo thứ tự bất kỳ, vào thời điểm bất kỳ**, tự đo khoảng cách lẫn nhau để suy ra tọa độ. Không cần đo tay lại mỗi lần dời anchor.
4. Mở rộng lên nhiều tag (nhiều drone).
5. Tích hợp PX4. **Hiện chưa đụng vào PX4**, xem mục 10.

### Ngoài phạm vi (hiện tại)

- Không sửa hay build lại firmware PX4.
- Không dùng TDoA.
- Không dùng BLE trên DWM1001 (tắt để tránh jitter).

---

## 2. Hiện trạng

### Phần cứng

| Vai trò | Label | J-Link S/N | Cổng serial cố định |
|---|---|---|---|
| Anchor (thử nghiệm) | DW431E | 000760163298 | `/dev/serial/by-id/usb-SEGGER_J-Link_000760163298-if00` |
| Tag (thử nghiệm) | DW0E9D | 000760163359 | `/dev/serial/by-id/usb-SEGGER_J-Link_000760163359-if00` |
| Anchor 2 | 2 | 000760163358 | `/dev/serial/by-id/usb-SEGGER_J-Link_000760163358-if00` |
| Flight controller | PX4 FMU v6C | – | `/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00` |

`./flash.py --list` luôn cho biết board nào đang cắm và đang chạy gì.

- Companion computer: **Raspberry Pi 4**.
- 4 anchor treo **cùng một độ cao**.
- Module: DWM1001C = nRF52832 (Cortex-M4F, 512 KB flash, 64 KB RAM) + DW1000 + gia tốc kế LIS2DH12.
- Cả hai board đang chạy PANS v1.3.1. Flash **không bị khóa đọc** (APPROTECT = `0xFFFFFFFF`).
- **Lưu ý:** tên `ttyACMx` thay đổi sau mỗi lần cắm lại. Luôn dùng đường dẫn `/dev/serial/by-id/...`.

### Đã xong (Giai đoạn 0)

- [x] Tải tài liệu vào [docs/](docs/): DWM1001 datasheet v1.6, Firmware API Guide v2.2, Firmware User Guide v1.3, DW1000 datasheet v2.10, DW1000 Driver API Guide v2.4
- [x] Sao lưu toàn bộ flash và UICR của cả 2 board vào [backup/](backup/), có `SHA256SUMS`
- [x] Công cụ: SEGGER J-Link V9.90 (`~/tools/jlink`, có symlink `/opt/SEGGER/JLink`), nrfutil 8.2.1 + nrfutil-device 2.23.0 (`~/tools/nrfutil`), arm-none-eabi-gcc 13.2.1, pyserial, pymavlink. Đã cài udev rule cho J-Link.
- [x] Clone [dwm1001-examples/](dwm1001-examples/) của Decawave. Repo có kèm nRF5 SDK 14.2.0 và driver DW1000 2.04, nhưng chỉ có project cho SES/Keil (không có Makefile) và chỉ có ví dụ **SS-TWR**.

### Còn thiếu (phải tải bằng trình duyệt vì Qorvo chặn tải tự động)

- DW1000 User Manual (mô tả thanh ghi, cần cho tối ưu)
- Datasheet và schematic của DWM1001-DEV
- Gói phần mềm DWM1001 (có file hex PANS chính thức để nạp lại cho board bất kỳ)

---

## 3. Các quyết định thiết kế

| # | Quyết định | Lý do |
|---|---|---|
| D1 | **DS-TWR** do tag tính, không dùng SS-TWR | Tự bù lệch xung nhịp giữa hai board. SS-TWR có sai số tăng theo thời gian trả lời |
| D2 | Tag xuất **khoảng cách thô, mỗi lần đo một gói** | EKF fuse từng khoảng cách: vẫn chạy khi mất anchor, lọc outlier theo từng anchor, không phải sửa firmware khi thêm anchor |
| D3 | **Mỗi khung 10 ms tag chỉ đo 1 anchor**, xoay vòng | EKF2 của PX4 chỉ nhận ~100 khoảng cách/giây cho tất cả beacon (mục 10). Đo nhiều hơn là phí. Mỗi anchor còn 25 Hz, còn dư thời gian cho tag khác |
| D4 | Timestamp theo **đồng hồ của tag**, lấy lúc đo | Jitter của UART/USB cỡ vài ms. Companion tự ước lượng độ lệch và độ trôi giữa hai đồng hồ |
| D5 | Tag **không biết tọa độ anchor**, chỉ gửi `anchor_id` | Tọa độ do companion giải từ dữ liệu tự calib và lưu ở đó |
| D6 | ~~Anchor tự calib với nhau, không phụ thuộc tag~~ → **Trước mắt (5a): tag điều phối việc calib**; anchor tự quản lý (5b) để sau | Tag hỏi lần lượt từng cặp nên không bao giờ phát trùng, không cần khung TDMA. Đổi lại cần tag đang chạy, và chỉ hợp 1 tag |
| D7 | **Độ cao anchor (z) nhập tay**, chỉ tự tính x, y | Anchor gần cùng độ cao thì z không giải được từ khoảng cách (đối xứng gương). Biết z thì còn dư phép đo để phát hiện lỗi |
| D8 | **Độ trễ anten calib một lần cho mỗi board**, lưu flash | Là đặc tính phần cứng, không đổi khi dời anchor. Không thể giải cùng lúc với tọa độ (6 phương trình, 9 ẩn) |
| D9 | Radio: kênh 5, 6.8 Mbps, PRF 64 MHz, preamble 128, PAC 8 | Cấu hình nhanh nhất, đã có sẵn trong ví dụ Decawave |
| D10 | Build bằng **Makefile + GCC**, không dùng SES/Keil | Quản lý trong repo, tự động hóa được, dùng toolchain có sẵn |
| D11 | UART **115200 baud**, nhị phân, có CRC | Đã thử 921600: mất ~18% gói (lỗi CRC) qua J-Link OB. 100 Hz chỉ cần ~2,6 KB/s, 115200 chở ~11,5 KB/s |
| D12 | Giải tọa độ anchor **trên companion**, không trên nRF52 | Dễ debug, dễ vẽ kết quả, dễ đổi thuật toán |

---

## 4. Sơ đồ đánh số anchor (quy ước chung)

Đánh số **1, 2, 3, 4 giống label của PANS**. Số này được chọn lúc nạp bằng `flash.py` và nằm trong mọi gói dữ liệu (`anchor_id`).

```
              nhìn từ trên xuống (anchor treo cùng độ cao ở 4 góc)

        A4 ●───────────────────────────────● A3
           │                               │
           │                               │
      +y ↑ │          khu vực bay          │
           │                               │
           │                               │
        A1 ●───────────────────────────────● A2
        (0,0)            trục +x →
```

| Anchor | Vị trí | Vai trò trong hệ tọa độ tự calib | Địa chỉ UWB |
|---|---|---|---|
| **A1** | Góc gốc | Gốc tọa độ (0, 0). ID nhỏ nhất nên là **trưởng nhóm** khi có mặt | `0xA001` |
| **A2** | Góc kề A1 | Xác định trục **+x** (nằm trên trục x) | `0xA002` |
| **A3** | Góc chéo với A1 | Được giải ra từ các khoảng cách | `0xA003` |
| **A4** | Góc kề A1, còn lại | Xác định phía **+y** (loại nghiệm gương) | `0xA004` |

- Đi theo thứ tự **1 → 2 → 3 → 4 là ngược chiều kim đồng hồ** khi nhìn từ trên xuống. Treo sai thứ tự (ví dụ đổi chỗ A2 và A4) thì hệ trục bị lật gương.
- **Tag** cũng đánh số từ 1 (địa chỉ `0x7001`, `0x7002`, ...).
- Nên dán nhãn số lên từng board cho khỏi nhầm.

---

## 5. Kiến trúc tổng thể

```
 ┌──────────── Anchor A1..A4 (treo 4 góc) ────────────┐
 │ - Trả lời DS-TWR từ tag                            │
 │ - Anchor ID nhỏ nhất đang hoạt động = TRƯỞNG NHÓM  │
 │   → phát BEACON mở đầu mỗi khung 10 ms             │
 │ - Tự đo cặp anchor–anchor, lưu bảng vào flash      │
 └───────────────▲────────────────────────────────────┘
                 │ UWB
 ┌───────────────┴──── Tag (trên drone) ──────────────┐
 │ - Đồng bộ theo BEACON, đo DS-TWR trong khe của mình│
 │ - Nghe bảng khoảng cách anchor do trưởng nhóm phát │
 │ - Xuất ra UART: RANGE, ANCHOR_PAIR, STATUS         │
 └───────────────┬────────────────────────────────────┘
                 │ UART 921600, nhị phân
 ┌───────────────▼──── Companion (host/) ─────────────┐
 │ - Đọc gói, kiểm tra CRC, đồng bộ thời gian         │
 │ - Giải tọa độ anchor từ bảng khoảng cách           │
 │ - Ghi log, đo tần số / nhiễu / mất gói             │
 │ - (Sau này) gửi sang PX4                           │
 └────────────────────────────────────────────────────┘
```

---

## 6. Khung thời gian (TDMA)

### Khung 10 ms *(ước lượng, chỉnh lại sau khi đo ở Giai đoạn 2)*

```
0        0.5               3.0                    6.5                    10 ms
|BEACON | Khe quản lý (M)  | Khe tag 1 (T1)       | Khe tag 2 (T2)       |
  trưởng   HELLO, hoặc 1      1 lần DS-TWR tới      1 lần DS-TWR tới
  nhóm     lần đo anchor–     anchor (k mod N)      anchor (k mod N)
           anchor
```

- `k` là số thứ tự khung, nằm trong BEACON. Ở khung `k`, tag đo anchor có thứ tự `k mod N`.
- **Khe quản lý thuộc về một anchor theo vòng** (`k mod N_max`), nên các anchor đã đồng bộ không phát trùng nhau.
- Khi **chưa có tag nào**, các khe tag cũng dùng cho đo anchor–anchor để calib nhanh hơn.
- Mỗi biên khe có khoảng đệm 100–200 µs *(ước lượng)*.

### Ngân sách thời gian của một lần DS-TWR *(ước lượng)*

| Thành phần | Thời gian |
|---|---|
| 1 frame (preamble 128 + SFD + PHR + ~16 byte payload @ 6.8 Mbps) | ~0.2 ms |
| Thời gian quay vòng của vi điều khiển (ngắt + SPI 8 MHz + hẹn giờ phát) | 0.3–0.5 ms |
| 4 frame + 3 lần quay vòng | **~2–2.5 ms** |

### Đồng bộ

- Anchor và tag lấy thời điểm nhận BEACON (timestamp DW1000) làm mốc khung, rồi hẹn giờ các khe bằng timer của nRF52. Độ chính xác cỡ chục µs là đủ.
- Mất BEACON quá **3 khung**: nếu là anchor thì chuyển sang bầu trưởng nhóm, nếu là tag thì ngừng phát và chờ.

---

## 7. Giao thức radio (UWB)

Header theo frame dữ liệu IEEE 802.15.4, giống ví dụ Decawave (10 byte): frame control (2), seq (1), PAN ID (2), địa chỉ đích (2), địa chỉ nguồn (2), mã chức năng (1). Địa chỉ broadcast là `0xFFFF`.

| Mã | Tên | Hướng | Nội dung chính |
|---|---|---|---|
| `0x10` | BEACON | Trưởng nhóm → tất cả | `frame_no`, danh sách anchor đang hoạt động (bitmask), phiên bản bảng |
| `0x11` | HELLO | Anchor → tất cả | `id`, trạng thái, bitmask các cặp đã có, phiên bản bảng |
| `0x12` | TABLE | Trưởng nhóm → tất cả | Bảng khoảng cách anchor (tối đa 6 cặp × ~8 byte). Phát ~1 lần/giây |
| `0x20` | POLL | Bên đo → bên được đo | – (lưu T1 = thời điểm phát) |
| `0x21` | RESP | Bên được đo → bên đo | T2 (nhận POLL), T3 (phát RESP, biết trước nhờ phát hẹn giờ) |
| `0x22` | FINAL | Bên đo → bên được đo | – (lưu T4 = nhận RESP, T5 = phát FINAL) |
| `0x23` | REPORT | Bên được đo → bên đo | T6 (nhận FINAL) |
| `0x31` | PAIR_RESULT | Anchor → tất cả | `id_a`, `id_b`, trung vị khoảng cách (mm), độ lệch chuẩn (mm), số mẫu, phiên bản |

### Công thức DS-TWR bất đối xứng (bên đo tự tính)

```
Ra = T4 − T1   (vòng của bên đo)       Da = T3 − T2   (trễ trả lời của bên được đo)
Rb = T6 − T3   (vòng của bên được đo)  Db = T5 − T4   (trễ trả lời của bên đo)

ToF = (Ra·Rb − Da·Db) / (Ra + Rb + Da + Db)
khoảng cách = ToF × c
```

Cùng một trình tự POLL → RESP → FINAL → REPORT được dùng cho cả **tag → anchor** và **anchor → anchor**.

---

## 8. Giao thức UART (tag → companion)

### Khung chung (little-endian)

```
| 0xAA | 0x55 | len (1) | type (1) | payload (len byte) | crc16 (2) |
```

CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`), tính trên `len`, `type` và `payload`.

### `0x01` RANGE: mỗi lần tag đo xong một anchor

| Trường | Kiểu | Ghi chú |
|---|---|---|
| `seq` | uint16 | Tăng mỗi gói, để phát hiện mất gói |
| `tag_id` | uint16 | |
| `anchor_id` | uint16 | 1–4 theo sơ đồ mục 4. Nhỏ hơn 256 nên khớp `beacon_id` (uint8) của PX4 |
| `t_us` | uint32 | Thời điểm đo theo đồng hồ tag, đếm tăng liên tục, tràn sau ~71 phút |
| `range_mm` | int32 | **Có dấu**: chưa calib có thể ra âm, không cắt bỏ |
| `rx_power_cdbm` | int16 | Công suất thu, dBm × 100 |
| `fp_power_cdbm` | int16 | Công suất đường đầu tiên, dBm × 100 |
| `flags` | uint8 | bit0 timeout, bit1 lỗi frame, bit2 nghi NLOS, bit3 khoảng cách không hợp lệ |

Payload 19 byte, cả gói 25 byte. Ở 100 Hz là ~2.5 KB/s, chỉ chiếm ~3% băng thông 921600 baud.

### `0x02` ANCHOR_PAIR: tag chuyển tiếp bảng khoảng cách anchor

| Trường | Kiểu |
|---|---|
| `table_version` | uint16 |
| `id_a`, `id_b` | uint16, uint16 |
| `dist_mm` (trung vị) | int32 |
| `std_mm` | uint16 |
| `n_samples` | uint16 |

Gửi khi bảng thay đổi, và nhắc lại mỗi ~1 giây.

### `0x03` STATUS: 1 Hz

`frame_no`, `leader_id`, bitmask anchor đang hoạt động, bitmask cặp đã đo, số lần đo thành công và thất bại trong giây vừa qua, phiên bản firmware.

### `0x80`–`0x8F`: lệnh từ companion → tag (dự phòng)

Ví dụ: yêu cầu đo lại toàn bộ cặp anchor, đổi cấu hình. Chưa làm ở giai đoạn đầu.

---

## 9. Tự calib anchor khi lên nguồn lần lượt

### Trạng thái của anchor

```
BOOT ──> LISTEN ──(nghe được BEACON)──────────────> JOIN ──> SURVEY ──> READY
           │                                          ▲                   │
           └─(300 ms không có BEACON)─> LÀM TRƯỞNG NHÓM┘                   │
                                                                          │
           ┌─────(anchor mới xuất hiện, hoặc cặp đã lưu lệch > ngưỡng)────┘
           ▼
         SURVEY (chỉ đo các cặp còn thiếu hoặc đã thay đổi)
```

- **LISTEN:** chỉ nghe, tối đa 300 ms. Nghe được BEACON thì vào nhóm. Không nghe thấy gì thì tự làm trưởng nhóm.
- **Bầu trưởng nhóm:** trưởng nhóm nghe thấy BEACON từ anchor có ID nhỏ hơn thì nhường lại. Trưởng nhóm mất tích quá 3 khung thì anchor có ID nhỏ nhất còn lại lên thay.
- **SURVEY:** đo các cặp còn thiếu trong khe quản lý của mình, mỗi cặp 100–200 mẫu rồi lấy trung vị, sau đó phát `PAIR_RESULT`. Mọi anchor đều giữ bảng đầy đủ.

### Ví dụ thứ tự bật nguồn

```
t=0s    A3 bật  → không có BEACON → làm trưởng nhóm
t=40s   A1 bật  → nghe BEACON → vào nhóm → ID nhỏ hơn nên thành trưởng nhóm mới
                → đo A1↔A3                               [1/6 cặp]
t=90s   A4 bật  → đo A4↔A1, A4↔A3                        [3/6]  ← 3 anchor: định vị được
t=200s  A2 bật  → đo A2↔A1, A2↔A3, A2↔A4                 [6/6]  ← có dư để kiểm tra lỗi
t=300s  Tag bật → nghe TABLE → chuyển ra UART → companion giải tọa độ
```

Thời gian đo một cặp: 100–200 mẫu, mỗi khung 1 mẫu thì ~1–2 giây. Khi chưa có tag thì dùng thêm khe tag, nhanh hơn *(ước lượng)*.

### Khởi động lại (thay pin, mất điện)

1. Đọc bảng đã lưu trong flash.
2. Đo nhanh lại các cặp của mình, ~20 mẫu mỗi cặp.
3. Lệch **dưới 5 cm**: giữ bảng cũ.
4. Lệch **trên 5 cm**: coi như anchor đã bị dời, đo lại đủ 3 cặp của nó và tăng `table_version`.

### Giải tọa độ (companion)

1. Đầu vào: bảng khoảng cách, độ cao z của từng anchor (file cấu hình), độ trễ anten đã được bù trong firmware.
2. Khoảng cách ngang: `d_ngang = √(d² − Δz²)`.
3. Chọn hệ trục theo sơ đồ mục 4: A1 = (0, 0), A2 trên trục +x, A4 ở phía +y (loại nghiệm gương). Vì 4 anchor cùng độ cao nên Δz = 0 và bài toán thuần 2D.
4. Ước lượng ban đầu bằng hình học tam giác hoặc MDS, rồi tối ưu Levenberg–Marquardt trên toàn bộ các cặp.
5. Kiểm tra: RMS sai số dư < 5 cm, A1/A2/A4 không gần thẳng hàng, mọi bộ ba thỏa bất đẳng thức tam giác. Không đạt thì báo cặp bị nghi bị che.
6. Hướng so với hướng Bắc: đo một lần góc của cạnh A1→A2 bằng la bàn, ghi vào file cấu hình. Chỉ cần khi tích hợp PX4.

### Quy tắc an toàn

- **Đang bay (armed) thì không thay tọa độ anchor.** Bảng mới chỉ được áp dụng sau khi disarm.
- Đang bay mà phát hiện anchor bị dời: bỏ anchor đó khỏi việc fuse, chạy tiếp bằng các anchor còn lại.

### Lưu trong flash của từng board

```c
struct node_cfg {          // ghi một lần khi calib board
    uint32_t magic, version;
    uint16_t node_id;      // anchor 1..4 theo sơ đồ mục 4, tag 1..
    uint8_t  role;         // ANCHOR / TAG
    uint8_t  n_anchors_expected;
    uint16_t tx_ant_dly, rx_ant_dly;
};
struct pair_table {        // chỉ anchor, cập nhật khi có thay đổi
    uint32_t magic;
    uint16_t version;
    struct { uint16_t a, b; int32_t dist_mm; uint16_t std_mm, n; } pair[6];
};
```

Chỉ ghi flash khi bảng thật sự thay đổi, để tránh mòn flash.

---

## 10. Tích hợp PX4 (đã khảo sát, **chưa làm**)

### Firmware trên board

PX4 **1.18.0**, nhánh `feature/rate-dob` (commit `7cc4bc18`), build ngày 22/10/2026 từ `/home/islab/sim/PX4-Autopilot`, biến thể `px4_fmu-v6c_default`.

### Ranging beacon trong EKF2 (fuse khoảng cách trực tiếp)

| Điểm | Kết quả |
|---|---|
| Code fuse có trong firmware | ✅ `CONFIG_EKF2_RANGING_BEACON=y`, có tham số `EKF2_RNGBC_CTRL/DELAY/GATE/NOISE` |
| Nhận `RANGING_BEACON` qua MAVLink | ❌ Firmware build với `CONFIG_MAVLINK_DIALECT="common"`, trong khi message chỉ có trong `development`. Đã gửi thử 20 gói: `listener ranging_beacon` báo `never published` |
| Qua uXRCE-DDS | ❌ `ranging_beacon` không có trong `dds_topics.yaml` |
| Message MAVLink | `RANGING_BEACON` id 513, đánh dấu **WIP** |

**Giới hạn đọc được từ code:**

- Tất cả beacon dùng chung một bộ đệm, mẫu đến nhanh hơn chu kỳ EKF (10 ms) bị bỏ, nên tổng cộng **~100 khoảng cách/giây** (đây là lý do của D3).
- uORB `ranging_beacon` có hàng đợi 1 phần tử.
- `K(pos z) = 0`: khoảng cách không sửa độ cao. Cần baro hoặc rangefinder.
- Tọa độ beacon tính bằng lat/lon/alt, bỏ qua `alt_type`. Trong nhà phải đặt global origin.
- `EKF2_RNGBC_NOISE` vừa là giá trị mặc định vừa là mức sàn (mặc định 30 m, phải chỉnh xuống ~0.1–0.2 m).
- Không có tham số bù vị trí lắp anten.
- Timestamp chỉ đúng khi companion trả lời gói `TIMESYNC` của PX4.

### Các hướng khi làm (để sau)

- **A.** Thêm `CONFIG_MAVLINK_DIALECT="development"` vào `boards/px4/fmu-v6c/default.px4board`, build lại. Companion gửi `RANGING_BEACON`. Thử trên SITL trước (SITL đã dùng dialect `development`).
- **B.** External Vision: companion tự fuse khoảng cách, gửi `ODOMETRY` 100 Hz kèm covariance. Không cần sửa PX4.
- **C.** Viết driver PX4 đọc tag trực tiếp qua UART, publish uORB `ranging_beacon`. Không cần companion.

Gói `RANGE` (mục 8) ánh xạ thẳng sang `RANGING_BEACON`, nên chọn hướng nào cũng **không phải sửa firmware tag**.

---

## 11. Cấu trúc thư mục

```
UWB/
├── PLAN.md                 # file này
├── flash.py                # dò board đang cắm, chọn tag/anchor + số thứ tự, build và nạp
├── docs/                   # tài liệu PDF
├── backup/                 # sao lưu PANS của từng board (không xóa)
├── dwm1001-examples/       # repo Decawave gốc: firmware dùng SDK và driver DW1000 trong đây, không sửa
├── firmware/
│   ├── Makefile            # make → build/uwb.hex (một image cho mọi board)
│   ├── nrf52832.ld
│   └── src/
│       ├── main.c          # đọc cấu hình UICR, chọn vai trò
│       ├── anchor.c        # trả lời DS-TWR
│       ├── tag.c           # chu kỳ 10 ms, đo xoay vòng anchor 1..N, tính khoảng cách
│       ├── radio.c/.h      # cấu hình DW1000, định dạng frame, hằng số thời gian DS-TWR
│       ├── proto.c/.h      # gói UART + CRC
│       ├── config.c/.h     # cấu hình trong UICR
│       ├── dw_port.c       # SPI và các hàm nền driver DW1000 cần
│       ├── hw.c, uart.c    # clock, timer µs, LED, UART DMA
│       └── board.h         # sơ đồ chân DWM1001-DEV
└── host/                   # Python, chạy được trên máy này và Pi 4
    ├── uwbproto.py         # giải mã gói, dò board (dùng chung)
    ├── uwb_reader.py       # đọc tag, thống kê mỗi giây, ghi CSV
    ├── uwb_record.py       # ghi N giây → data/*.xlsx (thô, vị trí, calib anchor, biểu đồ) + phân tích nhiễu
    ├── uwb_plot.py         # vẽ matplotlib từ file .xlsx (mặc định file mới nhất)
    ├── survey_solver.py    # giải tọa độ anchor từ bảng tự calib; chạy riêng: --write ghi anchors.toml
    ├── uwb_bridge.py       # CHẠY TRÊN PI: chuyển nguyên dòng byte UART của tag ra TCP :7777
    ├── uwb_viewer.py       # CHẠY TRÊN LAPTOP: giải mã, tính vị trí 2D, web :8000
    ├── web/index.html      # bản đồ A1–A4, vòng tròn khoảng cách, vị trí + vệt tag, bảng anchor
    ├── anchors.toml        # tọa độ anchor (m), độ cao anchor và tag
```

### Theo dõi trên web

```
Tag ──UART──> Pi 4: uwb_bridge.py ──TCP :7777──> Laptop: uwb_viewer.py ──> trình duyệt http://localhost:8000
              (chỉ chuyển byte,                  (CRC, giải mã, tính vị trí,
               không phân tích)                   phục vụ web)
```

```bash
# Trên Pi
host/uwb_bridge.py
# Trên laptop
host/uwb_viewer.py --tcp <ip-cua-pi>:7777
# Khi tag cắm thẳng vào laptop (như hiện tại), bỏ bridge:
host/uwb_viewer.py
```

- Chỉ cần Python và `pyserial`, không thư viện ngoài, trang web chạy được khi không có Internet.
- Vị trí được giải bằng bình phương nhỏ nhất 2D từ khoảng cách mới nhất (< 0.3 s) tới mỗi anchor, cần ≥ 3 anchor. Khi mới có 1–2 anchor thì trang vẫn hiện vòng tròn khoảng cách.
- Sửa `anchors.toml` thì viewer tự đọc lại, không cần khởi động lại.
- Đây là phần để **theo dõi**. Dữ liệu cho EKF của drone vẫn là dòng 100 Hz qua UART trên Pi.

### Cách dùng

```bash
./flash.py --list                 # xem board nào đang cắm, đang chạy gì
./flash.py                        # nạp: chọn board → Tag/Anchor → số thứ tự (→ số anchor nếu là tag)
host/uwb_reader.py                # tự tìm tag và in thống kê mỗi giây
host/uwb_reader.py --csv log.csv  # ghi thêm từng phép đo ra CSV
```

- Board đang chạy PANS mà chưa có trong `backup/` thì `flash.py` **tự sao lưu trước khi xóa**.
- Vai trò, số thứ tự, số anchor và baud được ghi vào **UICR** (`0x10001080`), không nằm trong firmware. Muốn đổi vai trò chỉ cần chạy lại `flash.py`.
- LED: **xanh lá** nhấp nháy 1 Hz = đang chạy. **Xanh dương** đảo mỗi lần đo thành công. **Đỏ** nhấp nháy = chưa cấu hình hoặc DW1000 lỗi.

---

## 12. Các giai đoạn

### Giai đoạn 1 + 2: DS-TWR 1 tag – 1 anchor ở 100 Hz — *code xong, chờ chạy thử*

Bỏ bước nạp ví dụ SS-TWR gốc, viết thẳng firmware riêng (bare-metal, không FreeRTOS, không SoftDevice/BLE).

- [x] `firmware/Makefile` dùng GCC 13, dùng SDK 14.2 chỉ cho header, startup và linker. Build sạch, ~13 KB.
- [x] DS-TWR 4 bản tin (POLL → RESP → FINAL → REPORT), phát hẹn giờ, công thức bất đối xứng.
- [x] Tag: chu kỳ 10 ms bằng timer phần cứng, đo xoay vòng anchor 1..N, gửi `RANGE` cho mọi lần thử (kể cả thất bại, có cờ lỗi).
- [x] Gói `RANGE` / `STATUS` nhị phân, UART 921600, CRC. Công suất thu và công suất đường đầu tiên lấy từ chẩn đoán DW1000.
- [x] `flash.py`: dò board, chọn vai trò + số thứ tự, tự sao lưu PANS, nạp, kiểm tra lại qua gói STATUS.
- [x] `host/uwb_reader.py`: tần số, jitter, mất gói, trung bình ± độ lệch chuẩn theo từng anchor, CSV.
- [x] **Chạy thử trên phần cứng** (2026-11-01, tag 1 = S/N 000760163387, 4 anchor cắm nguồn riêng, đứng yên, 115200 baud):

  | Anchor | Thành công | Khoảng cách | Độ lệch chuẩn | RX / FP (dBm) |
  |---|---|---|---|---|
  | A1 | ~100% | 4,31 m | 17–19 mm | −79,2 / −81,4 |
  | A2 | **~70–80%** (lỗi frame, timeout) | 2,69 m | 14–18 mm | −79,2 / −82,6 |
  | A3 | 100% | 3,10–3,15 m | **39–72 mm** | −79,4 / −81,6 |
  | A4 | 100% | 5,10 m | 26–33 mm | −79,0 / −83,5 |

  Tổng 101 gói/s, chu kỳ 10,00 ms, jitter 0 µs, không mất gói.
- [x] Ghi 10 s đứng yên bằng `host/uwb_record.py` → `data/uwb_20261101_173821.xlsx`. Vị trí: σx 34 mm, σy 46 mm, CEP50 51 mm, R95 96 mm. Bỏ A3: σx 24, σy 18, CEP50 25, R95 52 mm. (Tọa độ anchor trong `anchors.toml` lúc đó vẫn là giá trị ví dụ, sai số dư ~51 cm, nên chỉ xét độ nhiễu, không xét độ chính xác tuyệt đối.)
- [ ] **A3 nhiễu kiểu nhảy giữa hai mức** cách nhau ~15 cm (histogram hai đỉnh, tự tương quan bậc 1 = −0,45), khả năng cao là đa đường (multipath). Thử dời A3 hoặc đổi board A3 với anchor khác.
- [ ] A1 (~85%) và A2 (~77%) **mất phản hồi** (chủ yếu timeout, A2 có thêm lỗi frame). Thử đổi vị trí hoặc nới thời gian chờ để biết do môi trường hay do firmware.
- [ ] Đo thời gian quay vòng thực tế, rút ngắn các hằng số trong `radio.h` (đang để rộng: 1000 µs mỗi lần trả lời).

Khác với thiết kế ban đầu: DW1000 được **polling** thanh ghi trạng thái qua SPI, không dùng ngắt. Đơn giản hơn và đủ nhanh cho 100 Hz; chuyển sang ngắt nếu đo thấy cần.

- ✅ **Đạt khi:** ≥ 95 lần đo/giây, mất < 1%, độ lệch chuẩn khi đứng yên < 5 cm, đã đo được độ trễ thực tế.

### Giai đoạn 3a: Dùng calib nhà máy trong OTP — *code xong (v0.2), chờ chạy thử*

Theo datasheet DWM1001 (2.1.3–2.1.4) và diễn đàn Qorvo: mỗi module được calib lúc sản xuất cho cấu hình radio của PANS (kênh 5, 6,8 Mbps, PRF 64, preamble 128, **trùng cấu hình của mình**). Driver Decawave chỉ tự áp dụng XTRIM và LDOTUNE.

- [x] Đọc OTP `0x1C` (độ trễ anten, Qorvo: dùng cùng giá trị cho TX và RX), `0x19` (công suất phát), `0x1E` byte 2 (PGDLY) **khi SPI còn chậm** (đọc ở 8 MHz ra `0xFFFFFFFF`).
- [x] Thứ tự ưu tiên độ trễ anten: UICR > OTP > mặc định 16436. Gói `STATUS` báo giá trị và nguồn.
- [ ] Chạy thử: so khoảng cách với thước. Cộng đồng báo dùng OTP xong vẫn lệch 17–30 cm (PANS còn bù range bias theo công suất thu), nên có thể vẫn cần 3b.

### Giai đoạn 3b: Calib độ trễ anten từng board

- Đo ở 2 khoảng cách đã biết (ví dụ 3.00 m và 5.00 m), tính `tx/rx_ant_dly` cho từng board.
- Ghi vào UICR qua `flash.py` (ô `CUSTOMER[2]` đã dành sẵn, hiện để mặc định 16436).
- ✅ **Đạt khi:** sai số trung bình < 3 cm ở cả hai khoảng cách, giữ nguyên sau khi khởi động lại.

### Giai đoạn 4: Khung TDMA với 4 anchor

- Cần thêm 2 board anchor (nạp và calib như Giai đoạn 3).
- BEACON, bầu trưởng nhóm, tag đo xoay vòng theo `frame_no`.
- ✅ **Đạt khi:** tổng ~100 khoảng cách/giây chia đều 4 anchor. Tắt trưởng nhóm thì nhóm tự phục hồi trong < 100 ms. Không có khung bị trùng.

### Giai đoạn 5a: Tự calib anchor do tag điều phối — *code xong (v0.2), chờ chạy thử*

```
khi đủ n anchor trả lời (và còn cặp chưa đo):
  tag tạm dừng đo 100 Hz
  với mỗi cặp (a, b) còn thiếu, a < b:
    tag ──SURVEY_REQ(b, 100)──> anchor a ──100 × DS-TWR──> anchor b
    tag <──PAIR_RESULT(trung vị, σ, số mẫu)── anchor a        (thử lại tối đa 3 lần)
    tag ──UART ANCHOR_PAIR──> máy tính
  tag đo tiếp; bảng được gửi lại mỗi 2 s
máy tính: survey_solver giải 2D (A1 gốc, A2 trục +x), viewer/record tự dùng tọa độ này
```

- Gói REPORT mang **thời gian chạy của anchor**. Thời gian này giảm tức là anchor vừa khởi động lại (có thể đã bị dời), nên tag xóa 3 cặp của anchor đó và đo lại. Anchor chỉ bị che tạm thời thì **không** kích hoạt calib lại.
- **Calib lại toàn bộ:** nút **"Calib lại toàn hệ"** trên viewer (v0.3: viewer gửi lệnh `CMD_RESURVEY 0x80` xuống tag qua UART, đi qua cả bridge trên Pi), hoặc reset tag. **Calib lại một anchor:** reset anchor đó.
- v0.3: ngưỡng một cặp hợp lệ hạ còn **20/100 mẫu** (lần chạy thử đầu, A2 chỉ đạt 50% nên kẹt ở ngưỡng 50); cặp lỗi thử lại sau **30 s** thay vì 10 s.
- v0.4: **không tự calib định kỳ nữa.** Chỉ calib khi: bật nguồn tag, một anchor khởi động lại, hoặc bấm nút trên viewer. Sau mỗi lượt, nếu đã có ≥ 2n − 3 cặp (5/6 với 4 anchor, đủ giải hình) thì dừng, trạng thái `SURVEY_PARTIAL` ("thiếu cặp nhưng giải được"). Chỉ khi chưa đủ để giải mới thử lại mỗi 30 s, tối đa 10 lượt. Lý do: mỗi lượt calib làm tag ngừng đo 1–2 s, không chấp nhận được khi bay.
- LED: tag sáng **đỏ** trong lúc khảo sát; anchor sáng đỏ khi đang đo tới anchor khác.
- Máy tính: `host/survey_solver.py [--write]`. Viewer có thẻ "Tọa độ anchor" hiện 6 cặp, sai số dư, cảnh báo treo sai thứ tự hoặc lệch góc. `anchors.toml`: `source = "auto"` (mặc định) hoặc `"manual"`.
- Giới hạn: trong lúc khảo sát (ước lượng ~3–5 s) không có dữ liệu vị trí; cần tag đang chạy; thiết kế cho 1 tag.

### Giai đoạn 5b: Anchor tự calib không cần tag (thiết kế gốc)

- HELLO, đo cặp anchor–anchor trong khe quản lý, `PAIR_RESULT`, `TABLE`, lưu flash, kiểm tra khi khởi động lại.
- Tag chuyển tiếp `ANCHOR_PAIR`. Viết `host/survey_solver.py`.
- ✅ **Đạt khi:**
  - Bật 4 anchor theo thứ tự bất kỳ, cách nhau bất kỳ, đều có đủ bảng 6 cặp.
  - Tọa độ giải ra lệch ±5 cm so với đo bằng thước *(mục tiêu, cần xác nhận)*.
  - Dời 1 anchor thì chỉ 3 cặp của nó được đo lại.
  - Thay pin nhưng không dời anchor thì bảng giữ nguyên.

### Mô phỏng EKF2 offline — `ekf/px4_ekf_sim.py`

Chạy lại khoảng cách đã ghi (`data/*.xlsx`) qua mô hình phần fuse ranging beacon của EKF2 (release/1.18). Bộ đệm quan sát, dời timestamp, R = max(var, NOISE²), cổng `test_ratio > 1`, `K(pos z) = 0` làm đúng như code PX4. Phần dự đoán thay bằng mô hình vận tốc không đổi (vì không có IMU), độ cao lấy từ tham chiếu giả với `EKF2_RNG_NOISE`. Xuất bản đồ, độ lệch theo thời gian kèm ±1σ, và innovation từng anchor kèm cổng.

Lần chạy đầu (`data/ekf_run.xlsx`, 10 s đứng yên, anchor là hình chữ nhật 4,01 × 5,18 m khớp từ 4 cạnh đo được):
- `EKF2_RNGBC_NOISE` = 30 m (mặc định PX4): đầu ra trông mượt (σ 9/13 mm) nhưng **σ ước lượng 2,7 m**, tức EKF gần như bỏ qua UWB. Bắt buộc phải chỉnh.
- `EKF2_RNGBC_NOISE` = 0,1 m: σx/σy 18/26 mm, σ ước lượng 28/23 mm (khớp với thực tế), không mẫu nào bị loại, không mẫu nào rơi ở bộ đệm.
- **Innovation lệch có hệ thống theo từng anchor** (A1, A3 khoảng +10 cm; A4 khoảng −15 cm), cùng nguồn gốc với sai số dư khoảng 20 cm: tọa độ anchor và/hoặc độ lệch khoảng cách chưa đúng. Cần sửa trước khi bay.

### Mô phỏng đường External Vision — `ekf/px4_ev_sim.py`

Pi giải vị trí 2D cho mỗi khoảng cách mới (≥ 3 anchor), gửi `ODOMETRY` kèm covariance σr²(HᵀH)⁻¹, rồi EKF2 fuse EV theo `ev_pos_control.cpp`: phương sai = max(phương sai gói, `EKF2_EVP_NOISE`², 0,01²), cổng 5σ cho từng trục, fuse x rồi y.

Lần chạy `data/ev_run.xlsx` (10 s đứng yên, 6/6 cặp, sai số dư bản đồ anchor 7,3 mm):
- Pi gửi 90 Hz, khoảng trống lớn nhất 20 ms. EKF: σx/σy **7/5 mm**, R95 18 mm, σ vận tốc khoảng 1 cm/s, không loại mẫu nào (khi đứng yên, không có IMU).
- **Độ lệch chung của tag khoảng −21 cm** trên mọi khoảng cách (giải vị trí kèm ẩn lệch: sai số dư 18,4 → 7,2 cm). Phép đo giữa anchor với anchor không bị ảnh hưởng, nên khả năng cao là độ trễ anten của tag. Hệ quả: vị trí bị dời khoảng 20 cm. Cách sửa: calib độ trễ anten của tag (3b), hoặc Pi ước lượng ẩn lệch này.

### Real-to-sim: nhiễu UWB thật trên x500 trong Gazebo — `sim/`

```
tag (đứng yên trên bàn) ─UART─> uwb_real2sim.py ─ODOMETRY 50 Hz─> PX4 SITL (bản build sẵn ở ~/sim/PX4-Autopilot)
                                 EV = vị trí thật Gazebo + (vị trí UWB − trung bình khi đứng yên)
```

- `sim/run_sitl.sh`: SITL airframe 4016 `gz_x500_lidar_down` (rangefinder hướng xuống), thư mục làm việc riêng `sim/rootfs`, không đụng tham số SITL của repo PX4.
- `sim/uwb_real2sim.py --setup` một lần (`EKF2_GPS_CTRL` 0, `EKF2_EV_CTRL` 1, `EKF2_HGT_REF` = rangefinder, không cần RC). Sau đó chạy test: offboard, arm, giữ 1,5 m trong `--hover` giây, hạ cánh, xuất CSV + PNG ra `data/real2sim_*`. `--replay file.xlsx` để dùng dữ liệu đã ghi thay cho tag.
- Lưu ý khi viết: tham số kiểu số nguyên gửi qua MAVLink phải đóng gói nguyên dãy bit vào trường float. Bản đồ anchor phải cố định **trước** khi lấy trung bình tĩnh và không được đổi khi đang bay (lần chạy đầu bị lệch 35 cm vì bảng tự calib tới sau).

Kết quả 2026-11-01 (tag live, 6 cặp, EV 50 Hz, hover 22 s ổn định): giữ vị trí ngang **RMS 2,4 cm, 95% 5,3 cm, max 7,0 cm**; EKF − thật RMS 1,5 cm. Độ cao EKF lệch độ cao thật khoảng −35 cm: do cách SITL đặt gốc độ cao và rangefinder, chưa tìm hiểu, không liên quan UWB. Giới hạn: chỉ đưa vào **nhiễu**; độ lệch tuyệt đối khoảng 20 cm của tag, yaw/la bàn trong nhà, gió và việc thân drone che tag không có trong mô phỏng.

### Triển khai trên Raspberry Pi với ArduPilot — `pi/`

Cấu trúc, cài đặt, lệnh chạy và danh sách kiểm tra hover nằm ở [pi/README.md](pi/README.md). Kiến trúc: `fc_router` (C++ cũ trong `~/dark-landing`, giữ nguyên) giữ cổng `/dev/ttyAMA5` nối SERIAL2 của FC, đẩy telemetry qua UDP tới QGC trên NUC và mở cổng UDP nội bộ; `pi/uwb/uwb_bridge.py` đọc tag, giải vị trí ngang, gửi ODOMETRY qua cổng nội bộ đó.

Ngày 2026-11-04: FC ArduCopter 4.7.0 (board Saolah743) cấu hình PosHold với UWB cho vị trí ngang, độ cao baro, GPS tắt (`pi/config/ardupilot_uwb_extnav.param`). Chạy thử với tag thật: FC báo `EKF3 IMU0 is using external nav data` và `origin set`. Phát hiện khi thử: `SERIAL2_OPTIONS` bit 10 bị ArduPilot 4.7 chặn arm (đã sửa về 0); `OSD_TYPE2 = 5` (MSP DisplayPort) còn sót từ lúc SERIAL2 dùng cho OSD; baro trôi ~0,8 m khi FC mới bật nên bridge lấy độ cao tương đối so với lúc nằm dưới đất. Chưa arm, chưa bay, chưa thử với QGC thật.

### Giai đoạn 6: Tích hợp PX4

Làm khi bạn sẵn sàng sửa PX4. Chọn hướng A, B hoặc C ở mục 10, thử SITL trước rồi mới bay thật.

### Giai đoạn 7: Nhiều tag

- Mỗi tag một khe. Với khung 10 ms *(ước lượng)*, 2 tag giữ được 100 Hz. Nhiều hơn thì giảm tần số mỗi tag hoặc rút ngắn khe.

---

## 13. Rủi ro

| Rủi ro | Ảnh hưởng | Cách giảm |
|---|---|---|
| ~~nRF5 SDK 14.2 (2017) không build được với GCC 13~~ | – | **Đã xử lý:** chỉ dùng header/startup của SDK, build sạch |
| ~~J-Link OB không chạy được 921600 baud ổn định~~ | – | **Đã gặp và xử lý:** chuyển mặc định sang 115200, hết lỗi CRC |
| Thời gian quay vòng thực tế lớn hơn ước lượng | Không đủ chỗ cho 2 tag mỗi khung | Đo ở Giai đoạn 2 rồi chỉnh lại khung |
| Bị che khuất (NLOS) trong phòng | Khoảng cách dài hơn thực tế, calib anchor sai | Gắn cờ NLOS theo công suất, lấy trung vị, kiểm tra sai số dư |
| **4 anchor cùng độ cao (đã chọn)** | Trục z của tag gần như không quan sát được từ UWB | Độ cao lấy từ baro/rangefinder. Đổi lại, tự calib anchor thành bài toán 2D, chắc chắn hơn |
| Treo anchor sai thứ tự 1–4 | Hệ trục bị lật gương | Theo sơ đồ mục 4, dán nhãn số lên board |
| Hai anchor cùng nghĩ mình là trưởng nhóm | Trùng BEACON | Quy tắc nhường cho ID nhỏ hơn, có thời gian chờ ngẫu nhiên |
| Mất PANS | Không quay lại được | Đã sao lưu. Nên tải gói phần mềm chính thức của Qorvo |

---

## 14. Câu hỏi còn mở

1. Bộ MDEK1001 có bao nhiêu board dùng được? Cần ít nhất 4 anchor và 1 tag.
2. ~~Anchor treo cùng độ cao hay khác nhau?~~ → **Cùng độ cao.**
3. Anchor dùng pin hay nguồn cố định? (Ảnh hưởng tới việc phải tối ưu điện năng.)
4. Cuối cùng có bao nhiêu tag (drone)?
5. ~~Companion computer?~~ → **Raspberry Pi 4.** Còn cần biết hệ điều hành (Raspberry Pi OS 64-bit / Ubuntu?).
6. Kích thước phòng hoặc khu vực bay? (Ảnh hưởng tới cấu hình preamble và tầm xa.)
7. ~~Gán `node_id` thế nào?~~ → **Chọn lúc nạp bằng `flash.py`**, ghi vào UICR, đánh số 1–4 theo sơ đồ mục 4.

---

## Tài liệu tham khảo

- [DWM1001 Firmware API Guide v2.2](docs/DWM1001_Firmware_API_Guide_v2.2.pdf): định dạng UART của PANS (`dwm_loc_get`, `lec`)
- [DW1000 Device Driver API Guide v2.4](docs/DW1000_Device_Driver_API_Guide_v2.4.pdf): API driver dùng cho firmware tự viết
- [DW1000 Datasheet v2.10](docs/DW1000_Datasheet_v2.10.pdf), [DWM1001 Datasheet v1.6](docs/DWM1001_Datasheet_v1.6.pdf)
- Ví dụ của Decawave: [dwm1001-examples/](dwm1001-examples/)
- PX4: [EKF2 tuning](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html), [External position estimation](https://docs.px4.io/main/en/ros/external_position_estimation.html), mã nguồn `src/modules/ekf2/EKF/aid_sources/ranging_beacon/`
