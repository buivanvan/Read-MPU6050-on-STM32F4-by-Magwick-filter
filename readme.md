# Ước lượng tư thế với STM32F407 và MPU6050

Firmware STM32F407VETx dùng thanh ghi RCC, GPIOB, TIM2 và I2C1 qua CMSIS để đọc MPU6050 ở 100 mẫu/s. Mỗi mẫu cập nhật bộ lọc Madgwick và ba góc roll, pitch, yaw. Target Keil chỉ biên dịch startup, CMSIS và mã ứng dụng. [Vòng lặp chính](Core/Src/main.c) · [Driver MPU6050](Core/Src/mpu6050.c) · [Bộ lọc Madgwick](Core/Src/madgwick.c)

## Đấu dây và chạy trên bo

1. Nối PB6 với SCL, PB7 với SDA, nối chung GND. Hai đường bus cần điện trở kéo lên 3,3 V (kiểm tra điện trở sẵn có trên module). Cấp nguồn phù hợp với module. Nối AD0 xuống GND cho địa chỉ mặc định `0x68`. Nếu AD0 ở mức cao, sửa `MPU_ADDRESS` thành `0x69`. Firmware thăm dò cờ dữ liệu qua I²C nên có thể để trống chân INT.
2. Mở [MPU6050.uvprojx](MDK-ARM/MPU6050.uvprojx), Build và nạp bằng Keil. Cấu hình chân và clock nằm trong mã C viết tay. Tệp CubeMX ban đầu chưa chứa các cấu hình này.
3. Giữ bo đứng yên trong pha `CALIBRATING` (200 mẫu, khoảng 2 giây). Firmware lấy trung bình gyro để bù bias và dùng gia tốc trung bình để khởi tạo roll/pitch với yaw ban đầu bằng 0.
4. Trong Keil Watch, theo dõi `imu_state`, `sample_count` và `attitude`. Khi trạng thái là `RUNNING`, bộ đếm tăng khoảng 100 lần/s. Các trường `roll_deg`, `pitch_deg`, `yaw_deg` tính bằng độ. `accel_g`, `gyro_rad_s`, `q` và `dt_us` cho phép kiểm tra dữ liệu đầu vào, quaternion và khoảng thời gian giữa các lần đọc. Khi đọc lỗi hoặc hết thời gian chờ, `error_count` tăng. Firmware thử khởi tạo lại sau 250 ms và hiệu chuẩn lại khi kết nối thành công.

**Cấu hình mặc định.** Driver dùng clock HSI/APB1 sau reset ở 16 MHz, I²C 100 kHz, TIM2 1 MHz. Cảm biến đặt ±2 g, ±250 °/s, DLPF mức 3 và bộ chia mẫu bằng 9. Trong `main.c`, `BETA = 0.08f` là điểm khởi đầu để thử trên bo. `accel_bias` mặc định bằng 0, `accel_lsb_per_g` bằng 16384 cho từng trục. Hai cảm biến dùng cùng hệ trục in trên module. Nếu thay clock APB1, tính lại cả timing I²C và prescaler TIM2. [Thanh ghi STM32F407, RM0090](https://www.st.com/resource/en/reference_manual/rm0090-stm32f407-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

## Chọn bộ lọc và xem đồ thị UART

Sửa `#define use_filter` ở đầu [main.c](Core/Src/main.c), build và nạp lại firmware.

| Giá trị | Dữ liệu được tính và gửi |
|---|---|
| `1` (mặc định) | Gia tốc và tốc độ góc đã hiệu chuẩn, kèm roll/pitch/yaw từ Madgwick. |
| `0` | Gia tốc và tốc độ góc đã hiệu chuẩn. Firmware bỏ qua khởi tạo và cập nhật Madgwick. |

Cả hai chế độ đều giữ bước đo bias gyro khi khởi động. Công tắc này điều khiển Madgwick trên STM32. DLPF bên trong MPU6050 vẫn dùng cấu hình mức 3.

Nối **PA2 (USART2_TX)** vào **RX của USB–UART mức logic 3,3 V** và nối chung GND. Firmware chỉ truyền, ở 115200 baud, 8N1. Mỗi hai mẫu hợp lệ, STM32 gửi một dòng CSV, tương đương khoảng 50 dòng/s. Chạy từ thư mục project, thay `COM3` bằng cổng thực tế.

```shell
python -m pip install pyserial matplotlib
python -m serial.tools.list_ports
python plot_mpu.py COM3
```

[plot_mpu.py](plot_mpu.py) hiển thị cửa sổ 10 giây gần nhất theo thời gian của TIM2, tự nhận chế độ từ mỗi bản tin. Hai biểu đồ đầu vẽ X/Y/Z của gia tốc (g) và tốc độ góc (rad/s). Khi bật Madgwick, biểu đồ thứ ba vẽ roll/pitch/yaw (độ). Dữ liệu bắt đầu xuất khi firmware hoàn tất hiệu chuẩn.

Dòng CSV lần lượt chứa `time_us,mode,ax,ay,az,gx,gy,gz`. Chế độ 1 nối thêm `roll,pitch,yaw`. Các trường trên dây là số nguyên, gia tốc tính bằng mg (1/1000 g), tốc độ góc bằng mrad/s và góc bằng 1/100 độ. Nếu đổi clock APB1, cập nhật cả `USART2->BRR` trong `uart_init()`.

## Từ thanh ghi I²C đến mẫu đo

1. Driver phát START, gửi địa chỉ ghi và số thanh ghi, rồi phát repeated START với địa chỉ đọc. Byte địa chỉ trên bus bằng địa chỉ 7 bit dịch trái một bit, ghép với bit đọc/ghi. Khi khởi động, driver đọc `WHO_AM_I` tại `0x75` và kiểm tra giá trị `0x68` ở cả hai địa chỉ AD0.
2. Xóa bit `SLEEP` trong `PWR_MGMT_1`. Chọn dải đo qua `GYRO_CONFIG` và `ACCEL_CONFIG`, băng thông qua `CONFIG.DLPF_CFG` và tốc độ lấy mẫu qua `SMPLRT_DIV`.
3. Mỗi khi có mẫu mới, đọc liền 14 byte từ `ACCEL_XOUT_H` (`0x3B`). Sáu byte đầu là gia tốc X/Y/Z, hai byte tiếp là nhiệt độ và sáu byte cuối là gyro X/Y/Z. Ghép từng cặp theo thứ tự byte cao trước, diễn giải thành số nguyên 16 bit có dấu. Chỉ cập nhật bộ lọc khi đọc thành công một mẫu mới. [Bản đồ thanh ghi MPU6050](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf)

Với từng trục, nếu lưu độ lệch tĩnh (bias) theo đơn vị đếm thô, phép đổi đơn vị là

```text
a [g]     = (acc_raw  - bias_acc)  / độ_nhạy_acc
ω [rad/s] = (gyro_raw - bias_gyro) / độ_nhạy_gyro × π/180
```

Ví dụ, ở dải ±2 g và ±250 °/s, độ nhạy danh định tương ứng là 16384 LSB/g và 131 LSB/(°/s). Khi đổi dải đo phải đổi cả hệ số chia. Sau khi hiệu chuẩn, đưa hai vectơ về cùng hệ trục của bo trước khi lọc. [Đặc tính MPU6050](https://invensense.tdk.com/wp-www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)

## Madgwick IMU tính tư thế như thế nào?

Bộ lọc duy trì quaternion đơn vị `q = (w, x, y, z)` biểu diễn tư thế của bo. Gyro đo tốc độ quay nên bám được chuyển động nhanh, nhưng sai số tích phân làm góc trôi dần. Khi bo đứng yên hoặc chuyển động đều, gia tốc kế cung cấp hướng tham chiếu thẳng đứng để sửa độ nghiêng. Với sáu trục của MPU6050, roll và pitch có tham chiếu này. Yaw chỉ đo được tương đối so với tư thế ban đầu và sẽ trôi theo sai số gyro.

Với quy ước `q` quay từ hệ trục bo sang hệ quy chiếu, và gia tốc kế đọc +1 g theo Z khi bo nằm ngang, mỗi mẫu được xử lý như sau.

1. Tính đạo hàm dự đoán từ gyro bằng `q̇_gyro = ½ q ⊗ (0, ωx, ωy, ωz)`, với `⊗` là phép nhân quaternion và `ω` tính bằng rad/s.
2. Chuẩn hóa gia tốc thành vectơ đơn vị `â`. Hướng thẳng đứng dự đoán trong hệ trục bo là `v(q) = (2(xz − wy), 2(wx + yz), 1 − 2(x² + y²))`. Lấy sai số `f = v(q) − â`.
3. Tính `s = Jᵀ f`, trong đó `J` là ma trận đạo hàm của `v(q)` theo `(w, x, y, z)`. Sửa tốc độ biến thiên tư thế bằng `q̇ = q̇_gyro − β s/‖s‖`. Nếu độ lớn gia tốc hoặc gradient gần 0, dùng riêng `q̇_gyro` cho mẫu đó để tránh chia cho 0.
4. Tích phân `q_mới = normalize(q + q̇ × Δt)`, với `Δt` tính bằng giây giữa hai mẫu. Quaternion này là đầu ra và cũng là trạng thái cho lần cập nhật kế tiếp. Có thể đổi sang roll, pitch, yaw để hiển thị. [Báo cáo thuật toán của Sebastian Madgwick, mục 3.3](https://x-io.co.uk/downloads/madgwick_internal_report.pdf)

## Các thông số cần hiệu chỉnh

Nên xác nhận dải đo, đơn vị, chiều trục và thời gian lấy mẫu trước khi chỉnh `β`.

| Thông số | Cách xác định trên bo | Dấu hiệu để kiểm tra |
|---|---|---|
| Dải đo và độ nhạy | Chọn dải bao phủ gia tốc và tốc độ quay lớn nhất dự kiến. Đọc lại cấu hình và dùng hệ số chia tương ứng. | Tín hiệu thô chạm giới hạn 16 bit khi chuyển động mạnh cho thấy dải quá hẹp. |
| Bias gyro X/Y/Z | Để bo đứng yên ở nhiệt độ làm việc, lấy trung bình nhiều mẫu trên từng trục rồi trừ giá trị đó trước khi đổi đơn vị. | Sau bù, tốc độ quay trung bình của cả ba trục khi đứng yên phải gần 0 °/s. |
| Bias và độ nhạy gia tốc kế | Đo tĩnh ở sáu tư thế, lần lượt hướng mỗi trục lên rồi xuống. Với mỗi trục, gọi hai giá trị thô là A₊ ở +1 g và A₋ ở −1 g. Bias bằng (A₊ + A₋)/2, độ nhạy đo được bằng (A₊ − A₋)/2 LSB/g. | Độ lớn vectơ gia tốc sau bù phải gần 1 g ở cả sáu tư thế. |
| Thứ tự và dấu các trục | Xác định chiều X/Y/Z trên bo. Áp dụng cùng phép đổi hệ trục thuận tay phải cho cả gia tốc kế và gyro. | Nghiêng hoặc xoay từng trục riêng biệt và đối chiếu trục thay đổi cùng dấu của nó. |
| Tần số lấy mẫu và Δt | Chọn tốc độ xuất mẫu của MPU6050 rồi đo khoảng thời gian giữa các mẫu mới bằng timer. Ví dụ, 100 Hz đều tương ứng 0,010 s/mẫu. | Ghi lại Δt khi hệ thống chạy đủ tải. Sai đơn vị hoặc bỏ sót mẫu sẽ làm sai góc tích phân. |
| Băng thông DLPF | Chọn băng thông theo chuyển động nhanh nhất cần theo dõi. Giảm băng thông nếu tín hiệu thô nhiễu do rung. | So sánh nhiễu khi đứng yên và độ trễ khi xoay nhanh sau mỗi lần chỉnh. |
| Hệ số Madgwick β | Giữ nguyên các cấu hình trên. Tăng β nếu góc nghiêng hội tụ quá chậm khi đặt bo trở lại mặt phẳng. Giảm β nếu gia tốc tịnh tiến làm góc nghiêng sai nhiều. | Thử cả động tác nghiêng rồi giữ yên và động tác trượt ngang trong khi giữ nguyên tư thế. Ghi β kèm tần số mẫu và băng thông đã thử. |
