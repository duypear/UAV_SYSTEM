# UAV Telemetry Ground Station (ESP32-C3 + nRF24L01 + Web Dashboard)

Hệ thống trạm mặt đất (Ground Station) chạy trên vi điều khiển **ESP32-C3** bằng framework **ESP-IDF** (tương thích v5.x & v6.x), được thiết kế để:
1. Thu nhận các gói tin Telemetry thời gian thực tốc độ cao (100 Hz) từ STM32F103 Flight Controller qua module **nRF24L01** (Kênh 80, 1 Mbps).
2. Giải mã đầy đủ các khung dữ liệu: Góc nghiêng (Roll, Pitch, Heading), Con quay hồi chuyển (Gyro), Lệnh điều khiển (RC Sticks), Động cơ (ESC 4 kênh), Trạng thái an toàn (Failsafe/Armed).
3. Phát mạng **WiFi SoftAP** và mở **Web Dashboard thời gian thực** (không cần Internet) có đồng hồ chân trời nhân tạo (Artificial Horizon), thanh đo động cơ, trạng thái máy bay.

---

## 1. Sơ đồ nối dây giữa ESP32-C3 và nRF24L01

> [!WARNING]
> Module nRF24L01 (đặc biệt loại có anten rời PA/LNA) **bắt buộc phải cấp nguồn 3.3V**. Cần hàn thêm tụ 10µF - 100µF giữa chân VCC và GND của nRF24 để chống sụt áp.

### Bảng nối dây chi tiết:

| Chân nRF24L01 | Tên chân | Nối với chân ESP32-C3 | Chức năng |
| :---: | :---: | :---: | :--- |
| **Pin 1** | **GND** | **GND** | Nối mass chung |
| **Pin 2** | **VCC** | **3.3V** | Nguồn 3.3V sạch |
| **Pin 3** | **CE** | **GPIO 3** | Chip Enable (kéo HIGH để nhận sóng) |
| **Pin 4** | **CSN** | **GPIO 7** | SPI Chip Select |
| **Pin 5** | **SCK** | **GPIO 4** | SPI Clock |
| **Pin 6** | **MOSI** | **GPIO 6** | SPI Data out từ ESP32 sang nRF24 |
| **Pin 7** | **MISO** | **GPIO 5** | SPI Data in từ nRF24 vào ESP32 |
| **Pin 8** | **IRQ** | *(Không cần nối)* | Đọc cờ ngắt qua SPI |

*(Nếu bạn dùng kit ESP32-C3 SuperMini hoặc DevKit khác, các chân GPIO 3, 4, 5, 6, 7 đều là GPIO thông dụng không vướng strapping pin).*

---

## 2. Cách mở và sử dụng trong VS Code

Project đã tích hợp sẵn cấu hình `.vscode/` (`tasks.json`, `settings.json`):

1. Mở thư mục `d:/UAVTELE/uav_ground_station` trong **VS Code**.
2. Đảm bảo bạn đã cài đặt extension **ESP-IDF** trong VS Code (hoặc mở Terminal có môi trường ESP-IDF).
3. Trong VS Code:
   * **Đặt target ESP32-C3 (nếu chưa đặt):** Bấm `Ctrl + Shift + P` $\rightarrow$ gõ `ESP-IDF: Set Espressif Device Target` $\rightarrow$ chọn `ESP32-C3`.
   * **Biên dịch (Build):** Bấm nút hình trụ **Build** ở thanh trạng thái dưới cùng, hoặc bấm `Ctrl + Shift + B` (chọn `ESP-IDF: Build`).
   * **Nạp code (Flash):** Cắm cáp USB nối ESP32-C3 vào máy tính, chọn đúng cổng COM rồi bấm nút **Flash**.
   * **Xem Log (Monitor):** Bấm nút **Monitor** để xem log thu nhận gói tin.

### Hoặc dùng dòng lệnh (ESP-IDF Terminal):
```bash
# Thiết lập target ESP32-C3
idf.py set-target esp32c3

# Biên dịch
idf.py build

# Nạp code và mở Serial Monitor
idf.py -p COMx flash monitor
```

---

## 3. Cách xem Web Dashboard trên điện thoại hoặc máy tính

1. Sau khi ESP32-C3 khởi động, nó sẽ tự động phát mạng WiFi:
   * **Tên WiFi (SSID):** `UAV-GroundStation`
   * **Mật khẩu:** `uavtelemetry`
2. Dùng điện thoại hoặc laptop kết nối vào mạng WiFi trên.
3. Mở trình duyệt web (Chrome / Safari / Edge) và truy cập địa chỉ:
   ```text
   http://192.168.4.1
   ```
4. Bạn sẽ thấy giao diện **UAV Ground Station Dashboard**:
   * Đồng hồ chân trời nhân tạo xoay trực quan theo góc Roll và Pitch của máy bay.
   * La bàn số hiển thị góc Heading.
   * 4 cột đo công suất động cơ ESC (1000 - 2000 µs).
   * Tọa độ vị trí cần gạt tay điều khiển RC.
   * Tần số nhận gói tin thực tế (Hz) và tỷ lệ mất gói (Packet Loss).
