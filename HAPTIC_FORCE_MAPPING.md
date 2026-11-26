# Phương trình quy đổi Lực PaXini → Điện áp Van ESP32

Tài liệu tóm tắt toàn bộ chuỗi công thức quy đổi từ lực đo được trên cảm biến **PaXini** (hoặc nhập thủ công trên GUI SuP/Ring) đến giá trị **DAC MCP4725** điều khiển 5 van khí nén **A, B, C, D, E** trên **ESP32**.

---

## 1. Luồng truyền dữ liệu tổng quát

```text
PaXini L / R (Fx, Fy, Fz [N])
   │
   ▼  (Host C# — 100 Hz)
Tổng hợp lực & tách 5 kênh van [FA, FB, FC, FD, FE]
   │
   ▼  Lượng tử hóa 0.1 N/LSB (0..255)
Frame nhị phân 20-byte CAN 0x300 (8-byte payload)
   │
   ▼  USB CDC → STM32 Master → UART2 (115200 8-N-1, chuyển tiếp nguyên trạng)
ESP32 Slave (haptic_core.cpp)
   │
   ▼  Hàm truyền F → Vout → DAC 12-bit
PCA9548A Mux (0x77) → 5× MCP4725 DAC → 5 Van ITV (A..E)
```

---

## 2. Tổng hợp lực từ 2 cảm biến PaXini (Host C#)

Nguồn: [HapticPayloadSynthesizer.cs](file:///C:/Users/nguye/OneDrive/Desktop/Control/CONTROL_GUI_CS/ControlGui.Core/Haptic/HapticPayloadSynthesizer.cs)

Khi cả 2 ngón Trái ($L$) và Phải ($R$) đều có dữ liệu mới ($\Delta t \le 0.25\text{ s}$):

$$
F_{z,\text{avg}} = \frac{F_{z,L} + F_{z,R}}{2}, \qquad
F_{x,\text{net}} = \frac{F_{x,L} - F_{x,R}}{2}, \qquad
F_{y,\text{net}} = \frac{F_{y,L} + F_{y,R}}{2}
$$

- Nếu chỉ có bên **Trái ($L$)**: $F_{z,\text{avg}} = F_{z,L}$, $F_{x,\text{net}} = F_{x,L}$, $F_{y,\text{net}} = F_{y,L}$.
- Nếu chỉ có bên **Phải ($R$)**: $F_{z,\text{avg}} = F_{z,R}$, $F_{x,\text{net}} = -F_{x,R}$, $F_{y,\text{net}} = F_{y,R}$.
- Nếu cả 2 ngón đều cũ / mất kết nối ($\Delta t > 0.25\text{ s}$): $F_{z,\text{avg}} = F_{x,\text{net}} = F_{y,\text{net}} = 0$.

---

## 3. Tách lực sang 5 van $F_A, F_B, F_C, F_D, F_E$

### 3.1. Trong `HapticPayloadSynthesizer.cs` (luồng PaXini mặc định)

$$
\begin{aligned}
F_A &= \max(0,\; F_{y,\text{net}}) \\
F_B &= \max(0,\; F_{z,\text{avg}}) \\
F_C &= \max(0,\; -F_{y,\text{net}}) \\
F_D &= \max(0,\; -F_{x,\text{net}}) \\
F_E &= \max(0,\; F_{x,\text{net}})
\end{aligned}
$$

### 3.2. Trong `SupRingMapper.cs` (quy ước bài báo `I01_3 DOF Pneumatic Tactile Display_v01.docx`)

Nguồn: [SupRingMapper.cs](file:///C:/Users/nguye/OneDrive/Desktop/Control/CONTROL_GUI_CS/ControlGui.Core/Haptic/SupRingMapper.cs)

$$
\begin{aligned}
F_A &= \max(0,\; -F_x) && \text{(Trượt xuống / Slide Down — Distal Tip)} \\
F_B &= \max(0,\; F_z)  && \text{(Kẹp chặt / Squeeze — Medial Mid)} \\
F_C &= \max(0,\; +F_x) && \text{(Trượt lên / Slide Up — Proximal Base)} \\
F_D &= \max(0,\; -F_y) && \text{(Kéo ra / Pull — Left Wall)} \\
F_E &= \max(0,\; +F_y) && \text{(Đẩy vào / Push — Right Wall)}
\end{aligned}
$$

---

## 4. Lượng tử hóa sang payload `CAN 0x300` (Host C# → STM32 → ESP32)

Mỗi lực $F_k$ ($k \in \{A,B,C,D,E\}$) được chuyển sang 1 byte nguyên không dấu (`0..255`) với độ phân giải $0.1\text{ N/LSB}$:

$$
b_k = \text{clamp}\!\Big(\text{round}(10 \cdot F_k),\; 0,\; 255\Big)
$$

Cấu trúc 8 byte payload của frame `CAN 0x300`:

| Byte | Nội dung | Công thức / Ý nghĩa |
|---|---|---|
| `payload[0]` | `b_A` | Byte lực van A (`0..255`, tức `0..25.5 N`) |
| `payload[1]` | `b_B` | Byte lực van B (`0..255`, tức `0..25.5 N`) |
| `payload[2]` | `b_C` | Byte lực van C (`0..255`, tức `0..25.5 N`) |
| `payload[3]` | `b_D` | Byte lực van D (`0..255`, tức `0..25.5 N`) |
| `payload[4]` | `b_E` | Byte lực van E (`0..255`, tức `0..25.5 N`) |
| `payload[5]` | `Reserved` | Mặc định `0x00` |
| `payload[6]` | `Status_Flags` | Bitmask: `0x01` L-contact, `0x02` R-contact, `0x04` Slip, `0x08` Tare |
| `payload[7]` | `Frame_Counter` | Bộ đếm tuần hoàn `0..255` |

> **Lưu ý**: STM32 master chỉ kiểm tra định dạng/CRC-8 và chuyển tiếp nguyên vẹn 20 byte qua UART2 xuống ESP32, không làm thay đổi giá trị $b_k$.

---

## 5. Phương trình quy đổi trong firmware ESP32

Nguồn: [haptic_core.cpp](file:///C:/Users/nguye/OneDrive/Desktop/Control/ESP32_FIRMWARE/src/haptic_core.cpp) và [config.h](file:///C:/Users/nguye/OneDrive/Desktop/Control/ESP32_FIRMWARE/include/config.h)

### 5.1. Khôi phục giá trị lực từ byte nhận được

$$
F_{k,\text{esp}} = 0.1 \cdot b_k \quad [\text{N}], \qquad F_{k,\text{esp}} \in [0,\; 25.5]\text{ N}
$$

### 5.2. Hàm truyền Lực → Điện áp van (`compute_valve_voltage`)

Đây là **hàm tuyến tính từng đoạn (piecewise linear)** gồm 3 vùng: **vùng chết (deadband)**, **vùng tuyến tính có điện áp nền (offset + gain)**, và **vùng bão hòa an toàn (saturation)**.

#### Các hằng số cấu hình trong `config.h`

| Hằng số | Giá trị | Ý nghĩa vật lý & kỹ thuật |
|---|---|---|
| `F_THRESH` ($f_{dz}$) | `0.20` N | **Ngưỡng vùng chết (deadband)**: lọc nhiễu không tải của PaXini; lực $\le 0.20\text{ N}$ bị coi như chưa tiếp xúc ($V = 0$). |
| `F_RATED` ($F_{\text{sat}}$) | `20.00` N | **Lực định mức bão hòa**: từ $20.0\text{ N}$ trở lên van đạt mức áp an toàn tối đa. |
| `V_MIN` ($V_0$) | `0.70` V | **Điện áp mở tối thiểu**: nhảy qua vùng chết cơ khí của van ITV và màng silicone để màng bắt đầu tiếp xúc nhẹ với da (tương ứng $P_0$ trong bài báo). |
| `V_MAX` | `2.20` V | **Giới hạn điện áp an toàn tuyệt đối**: chặn không cho van quá áp gây rách/thủng màng SuP (không bao giờ xuất tới `3.30 V`). |
| `V_WRITE_DELTA` | `0.015` V | **Băng trễ cập nhật I2C**: chỉ ghi lại DAC khi điện áp mới lệch $\ge 15\text{ mV}$ so với giá trị đang giữ để giảm tải bus I2C. |

#### Phương trình toán học đầy đủ

Đặt hệ số góc (độ lợi lực → điện áp) trên đoạn tuyến tính là:

$$
k = \frac{V_{\max} - V_{\min}}{F_{\text{rated}} - F_{\text{thresh}}}
  = \frac{2.20 - 0.70}{20.00 - 0.20}
  = \frac{1.50}{19.80}
  \approx 0.075758\text{ V/N}
$$

Khi đó, hàm quy đổi từ lực $F$ (N) sang điện áp đầu ra DAC $V_{\text{out}}(F)$ (V) là:

$$
V_{\text{out}}(F) =
\begin{cases}
0, & F \le 0.20\text{ N} \\[8pt]
V_{\min} + k\,(F - F_{\text{thresh}}) = 0.70 + 0.075758\,(F - 0.20) = 0.68485 + 0.075758\,F, & 0.20 < F < 20.00\text{ N} \\[8pt]
2.20, & F \ge 20.00\text{ N}
\end{cases}
$$

Hoặc viết gọn bằng hàm chuẩn hóa $\text{ratio} = \text{clamp}\!\left(\dfrac{F - F_{\text{thresh}}}{F_{\text{rated}} - F_{\text{thresh}}},\; 0,\; 1\right)$:

$$
V_{\text{out}}(F) =
\begin{cases}
0, & F \le F_{\text{thresh}} \\
V_{\min} + (V_{\max} - V_{\min}) \cdot \text{ratio}, & F > F_{\text{thresh}}
\end{cases}
$$

#### Giải thích ý nghĩa từng đoạn & liên hệ với bài báo `I01_v01.docx`

1. **Vùng $F \le 0.20\text{ N}$ ($V_{\text{out}} = 0\text{ V}$)**: Xả áp hoàn toàn về `0 V` để tránh van đóng/xả liên tục do nhiễu cảm biến khi ngón tay chưa chạm vật.
2. **Bước nhảy tại $F = 0.20^+\text{ N}$ ($V_{\text{out}} \approx 0.70\text{ V}$)**: Van ITV và màng silicone cần một mức áp ngưỡng tối thiểu mới bắt đầu phồng chạm vào đầu ngón tay. Nếu tăng tuyến tính từ `0 V`, cả đoạn `0 → 0.70 V` sẽ bị trễ/không cảm nhận được.
3. **Đoạn tuyến tính $0.20 < F < 20.00\text{ N}$**: Cứ mỗi **$1\text{ N}$ lực tăng thêm**, điện áp DAC tăng thêm **$\approx 75.76\text{ mV}$** (tương đương **$\approx 94$ bước DAC 12-bit**).
4. **Vùng $F \ge 20.00\text{ N}$ ($V_{\text{out}} = 2.20\text{ V}$)**: Kẹp cứng ở `2.20 V` nhằm giới hạn áp suất cực đại trong khoang SuP, bảo vệ màng silicone.
5. **Đối chiếu với phương trình trong bài báo `I01_v01.docx` (§II-D)**:
   - Bài báo viết tách 2 bước: $P_k = P_0 + G_j \cdot \text{clamp}(F_j - f_{dz},\; 0,\; F_{\text{sat},j} - f_{dz})$ và $u_k = (P_k - b_k)/a_k$.
   - Firmware ESP32 hiện tại **gộp chung 2 bước** thành một ánh xạ tuyến tính trực tiếp từ lực $F$ sang điện áp DAC $V_{\text{out}}(F)$, trong đó $f_{dz} = F_{\text{thresh}} = 0.20\text{ N}$, $F_{\text{sat}} = F_{\text{rated}} = 20.00\text{ N}$, $V_{\min} = 0.70\text{ V}$ đóng vai trò mức nền $P_0$, và độ dốc $k = 0.075758\text{ V/N}$ gộp chung độ lợi $G_j / a_k$.

### 5.3. Quy đổi sang thanh ghi DAC 12-bit MCP4725 (`voltage_to_dac`)

Với `VDD = 3.30 V` và `DAC_MAX = 4095`:

$$
\text{DAC}_{12\text{bit}} = \left\lfloor \frac{V_{\text{out}}}{3.30} \cdot 4095 \right\rfloor
$$

- Tại $V_{\text{out}} = 0.00\text{ V}$: $\text{DAC} = 0$
- Tại $V_{\text{out}} = 0.70\text{ V}$ ($F \approx 0.2+\text{ N}$): $\text{DAC} = 868$
- Tại $V_{\text{out}} = 2.20\text{ V}$ ($F \ge 20.0\text{ N}$): $\text{DAC} = 2730$

---

## 6. Bảng tra nhanh một số mốc lực

| Lực đầu vào $F_k$ (N) | Byte `b_k` | Lực tại ESP32 (N) | Điện áp $V_{\text{out}}$ (V) | Giá trị DAC 12-bit |
|---:|---:|---:|---:|---:|
| `0.0` | `0` | `0.0` | `0.000` | `0` |
| `0.2` | `2` | `0.2` | `0.000` | `0` |
| `0.3` | `3` | `0.3` | `0.708` | `878` |
| `1.0` | `10` | `1.0` | `0.761` | `943` |
| `5.0` | `50` | `5.0` | `1.064` | `1319` |
| `10.0` | `100` | `10.0` | `1.442` | `1789` |
| `15.0` | `150` | `15.0` | `1.821` | `2259` |
| `20.0` | `200` | `20.0` | `2.200` | `2730` |
| `25.5` | `255` | `25.5` | `2.200` | `2730` |
