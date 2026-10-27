# Fuzzy PID Attitude Control on PX4

**Thiết kế và tích hợp bộ điều khiển Fuzzy PID cho quadrotor trên nền tảng PX4 Autopilot.**

Dự án kết hợp mô phỏng trước trên **gazebo**, lập trình C++ trong PX4 và bước đầu thử nghiệm trên quadrotor sử dụng **MicoAir H743 V2 / STM32H743**.

Phần phát triển tập trung vào **vòng điều khiển tư thế (attitude outer loop)** trong `mc_att_control`: sử dụng logic mờ để điều chỉnh hệ số PID theo trạng thái sai lệch, sau đó tạo tốc độ góc đặt cho vòng điều khiển tốc độ góc sẵn có của PX4.

**Nhánh dự án:** [`feature/fuzzy-PID`](https://github.com/khoa-asta/PX4-Autopilot/tree/feature/fuzzy-PID). Đây là một bản phát triển từ PX4, với phạm vi đóng góp được mô tả bên dưới.

## Xem nhanh

| Nội dung | Liên kết |
| --- | --- |
| Báo cáo đồ án đầy đủ | [DATN.pdf](docs/fuzzy_pid_demo/DATN.pdf) |
| Video demo: tác động lực | [tac_dung_luc.mp4](docs/fuzzy_pid_demo/videos/tac_dung_luc.mp4) |
| Video demo: vật nặng | [vat_nang.mp4](docs/fuzzy_pid_demo/videos/vat_nang.mp4) |
| Mã nguồn phần điều khiển | [mc_att_control](src/modules/mc_att_control/) |


**Điểm chính:** triển khai bộ điều khiển Fuzzy PID cho vòng điều khiển attitude, tích hợp với quaternion attitude error và uORB, cấu hình qua PX4 parameters, build firmware cho STM32H743, và phân tích đáp ứng/dao động từ dữ liệu bay được trình bày trong báo cáo.

## 1. Bài toán và mục tiêu

Quadrotor chịu ảnh hưởng của thay đổi tải và nhiễu, trong khi chất lượng điều khiển phụ thuộc vào việc lựa chọn hệ số PID. Đồ án nghiên cứu bộ điều khiển tự chỉnh định bằng logic mờ nhằm cải thiện khả năng bám giá trị đặt và thích nghi với các điều kiện vận hành khác nhau.

Quy trình thực hiện gồm:

1. Tìm hiểu động lực học quadrotor và cấu trúc điều khiển phân tầng.
2. Xây dựng, mô phỏng và so sánh PID/Fuzzy PID trong MATLAB/Simulink.
3. Hiện thực thuật toán Fuzzy PID bằng C++ trong vòng điều khiển tư thế của PX4.
4. Build firmware, cấu hình phần cứng bằng QGroundControl và đánh giá bước đầu trên thiết bị thật.

## 2. Phạm vi đóng góp

| Hạng mục | Phần thực hiện trong dự án |
| --- | --- |
| Thuật toán Fuzzy PID | Bổ sung lớp `FuzzyAttitude`, chuẩn hóa hai đầu vào sai lệch và tốc độ biến thiên sai lệch, tính mức điều chỉnh Kp/Ki/Kd từ bảng luật mờ. |
| Tích hợp bộ điều khiển | Bổ sung ba bộ Fuzzy PID cho Roll/Pitch/Yaw vào `AttitudeControl`; sử dụng sai lệch tư thế quaternion để tạo body-rate setpoint. |
| Xử lý số và giới hạn | Bổ sung giới hạn tích phân, điều kiện chống windup khi đầu ra bão hòa, lọc tín hiệu nhánh D, giới hạn đầu ra và kiểm tra giá trị hữu hạn. |
| Tích hợp vào module PX4 | Đọc tốc độ góc từ uORB, truyền `dt` và các tham số vào bộ điều khiển; reset trạng thái khi cần, ví dụ sau EKF attitude reset hoặc khi vòng attitude ngừng hoạt động. |
| Cấu hình tham số | Bổ sung `MC_FUZZY_EN` và nhóm `MC_FZ_*` để cấu hình bộ điều khiển qua hệ thống parameters/QGroundControl. |
| Build | Cập nhật CMake để biên dịch lớp mới cùng thư viện `AttitudeControl`. |
| Mô phỏng và đánh giá | Trình bày mô hình MATLAB/Simulink, kết quả so sánh, cấu hình phần cứng, quá trình build và phân tích dữ liệu bay trong báo cáo. |

**Các thành phần kế thừa từ PX4:** NuttX RTOS, uORB, EKF2, quản lý chế độ bay, điều khiển vị trí, vòng điều khiển tốc độ góc, control allocation, drivers, MAVLink và hệ thống build. Dự án sử dụng và tích hợp với các thành phần này; việc phát triển chúng không thuộc phạm vi đóng góp cá nhân được trình bày ở đây.

Commit triển khai chính nêu trên thay đổi **8 file** trong `mc_att_control`, bao gồm hai file mới `FuzzyAttitude.cpp` và `FuzzyAttitude.hpp`. Các commit tài liệu/demo được lưu riêng trong lịch sử nhánh.

## 3. Nguyên lý và vị trí tích hợp

| Bước | Xử lý |
| --- | --- |
| Đầu vào | Tư thế hiện tại, tư thế đặt, tốc độ góc đo được và chu kỳ cập nhật `dt`. |
| Sai lệch tư thế | Sử dụng phần tính sai lệch quaternion và ưu tiên Roll/Pitch của PX4; tốc độ biến thiên sai lệch được xấp xỉ từ tốc độ góc thân. |
| Suy luận mờ | Hai đầu vào được chuẩn hóa; mỗi đầu vào có 7 tập mờ, kết hợp thành bảng 7 × 7 luật. Tính các mức điều chỉnh Kp/Ki/Kd bằng trọng số suy luận. |
| Điều khiển | Ba bộ Fuzzy PID tạo tốc độ góc đặt cho Roll/Pitch/Yaw, có xử lý tích phân, nhánh D và bão hòa đầu ra. |
| Đầu ra | Xuất `vehicle_rates_setpoint` qua uORB để vòng `mc_rate_control` sẵn có tiếp tục xử lý. |

`MC_FUZZY_EN = 0` chọn bộ điều khiển tư thế P theo quaternion sẵn có; `MC_FUZZY_EN = 1` chọn phần mở rộng Fuzzy PID. Phần mở rộng này không trực tiếp phát lệnh động cơ.

## 4. Các file quan trọng nên đọc

Thứ tự gợi ý để tìm hiểu phần đóng góp:

| Thứ tự | File | Nội dung cần xem |
| --- | --- | --- |
| 1 | [FuzzyAttitude.hpp](src/modules/mc_att_control/AttitudeControl/FuzzyAttitude.hpp) | Giao diện lớp, cấu hình và trạng thái của một trục điều khiển. |
| 2 | [FuzzyAttitude.cpp](src/modules/mc_att_control/AttitudeControl/FuzzyAttitude.cpp) | Hàm liên thuộc, bảng luật, suy luận mờ, cập nhật PID và xử lý giới hạn. |
| 3 | [AttitudeControl.cpp](src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp) | Ghép sai lệch quaternion với ba bộ Fuzzy PID; lựa chọn giữa bộ điều khiển gốc và phần mở rộng. |
| 4 | [AttitudeControl.hpp](src/modules/mc_att_control/AttitudeControl/AttitudeControl.hpp) | Các hàm cấu hình Fuzzy PID và thành phần lớp `AttitudeControl`. |
| 5 | [mc_att_control_main.cpp](src/modules/mc_att_control/mc_att_control_main.cpp) | Luồng thực thi module, đọc parameters/uORB, xử lý dữ liệu tốc độ góc và xuất rate setpoint. |
| 6 | [mc_att_control.hpp](src/modules/mc_att_control/mc_att_control.hpp) | Khai báo subscription và các PX4 parameters được sử dụng. |
| 7 | [mc_att_control_params.yaml](src/modules/mc_att_control/mc_att_control_params.yaml) | Tên, ý nghĩa, đơn vị, phạm vi và giá trị mặc định của tham số. |
| 8 | [AttitudeControl/CMakeLists.txt](src/modules/mc_att_control/AttitudeControl/CMakeLists.txt) | Đưa `FuzzyAttitude.cpp` vào quá trình build. |


## 5. Kết quả và bằng chứng

Các kết quả dưới đây được trình bày trong [báo cáo đồ án](docs/fuzzy_pid_demo/DATN.pdf). Số trang trong bảng là **số in trên trang báo cáo**, không phải số thứ tự trang của trình đọc PDF.

| Hạng mục | Kết quả được trình bày | Vị trí trong báo cáo |
| --- | --- | --- |
| Mô phỏng MATLAB/Simulink | Đồ thị vị trí, góc, sai lệch và quỹ đạo 3D để so sánh PID/Fuzzy PID trong các kịch bản mô phỏng. | Chương 4, trang 13–18, Hình 4.1–4.14. |
| Build cho phần cứng | Ảnh quá trình tạo firmware với target `micoair_h743-v2_default`. | Chương 6, trang 36, Hình 6.4. |
| Cấu hình và thử nghiệm | Hiệu chỉnh cảm biến, cấu hình cơ cấu chấp hành/chế độ bay và các đồ thị log về độ cao, góc, tốc độ góc. | Chương 6, trang 37–43, Hình 6.5–6.17. |
| Phân tích dao động | Phổ FFT tín hiệu cơ cấu chấp hành và vận tốc góc; thảo luận dao động tốc độ góc và nhiễu trong khoảng 90–100 Hz. | Trang 43–45, Hình 6.18–6.19 và mục 6.6. |

Kết quả thực nghiệm bộ điều khiển Fuzzy PID cải thiện đáp ứng trong các trường hợp mô phỏng đã khảo sát. Với thử nghiệm phần cứng, báo cáo ghi nhận khả năng bám tư thế nhưng vẫn còn dao động ở vòng tốc độ góc và ảnh hưởng của rung cơ khí.

**Giới hạn đánh giá:** Mới chỉ thực nghiệm với các quỹ đạo bay đơn giản chưa thực nghiệm và đánh giá chất lượng bộ điều khiển trong các môi trường khắc nhiệt

Hai video [tác động lực](docs/fuzzy_pid_demo/videos/tac_dung_luc.mp4) và [vật nặng](docs/fuzzy_pid_demo/videos/vat_nang.mp4), cùng [tài liệu demo](docs/fuzzy_pid_demo/videos/README.md), được lưu trong `docs/fuzzy_pid_demo/videos/`.

## 6. Môi trường và phần cứng

| Thành phần | Vai trò |
| --- | --- |
| C++, PX4 Autopilot, NuttX | Triển khai bộ điều khiển và chạy firmware trên flight controller. |
| Ubuntu / Ubuntu trong WSL2 | Môi trường build PX4; sử dụng terminal Ubuntu cho các lệnh build bên dưới. |
| Gazebo và QGroundControl | Môi trường SITL và giao diện cấu hình/giám sát PX4. |
| MATLAB/Simulink | Môi trường mô phỏng thuật toán trong báo cáo. |
| MicoAir H743 V2 / STM32H743 | Flight controller của mô hình thực nghiệm. |
| Khung quadrotor 7 inch, ESC MicoAir 4-in-1 55A | Nền tảng cơ khí và công suất được mô tả trong báo cáo. |

Phiên bản MATLAB, bộ tham số thử nghiệm hoàn chỉnh và các tệp mô hình Simulink cần được đối chiếu riêng khi tái lập phần mô phỏng. Các lệnh SITL dưới đây dùng mô hình Gazebo `x500` để khởi chạy phần mềm, không tái tạo nguyên trạng mô hình 7 inch trong báo cáo.

## 7. Hướng dẫn build và chạy

### 7.1. Cài đặt trên thiết bị

Sử dụng WSL2/ phần mềm giả lập Linux trên Window/ Hệ điều hành Linux/ MacOS:

```bash
git clone --branch feature/fuzzy-PID --recursive https://github.com/khoa-asta/PX4-Autopilot.git
cd PX4-Autopilot
git submodule update --init --recursive
git rev-parse --short HEAD
```


### 7.2. Chuẩn bị toolchain

Với Ubuntu mới chưa có môi trường PX4, làm theo [hướng dẫn môi trường Ubuntu của PX4](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html). Script có sẵn trong nhánh dự án:

```bash
bash Tools/setup/ubuntu.sh
```

Script của nhánh này nhắm tới Ubuntu LTS 22.04/24.04 và cài các công cụ build/mô phỏng cần thiết. Thực hiện yêu cầu khởi động lại nếu script thông báo. Máy đã có toolchain tương thích có thể chuyển sang bước build.

### 7.3. Hướng dẫn build và chạy mô phỏng Gazebo SITL:

Từ thư mục gốc repository:

```bash
make px4_sitl gz_x500
```

Mở QGroundControl để kết nối với SITL. Tham khảo thêm [hướng dẫn Gazebo của PX4](https://docs.px4.io/main/en/sim_gazebo_gz/).

Lệnh này là bước khởi chạy môi trường phần mềm. Để đánh giá đúng phần Fuzzy PID, cần cấu hình các tham số ở mục tiếp theo và chọn chế độ có sử dụng vòng điều khiển tư thế, chẳng hạn Stabilized, Altitude hoặc Position. Acro đi theo đường điều khiển tốc độ góc nên không dùng để kiểm chứng phần mở rộng vòng attitude này.

### 7.4. Kiểm tra và lựa chọn bộ điều khiển

Trong PX4 shell của SITL, hoặc MAVLink Console trong QGroundControl:

```text
param show MC_FUZZY_EN
param show MC_FZ_*
```

| Nhóm tham số | Ý nghĩa |
| --- | --- |
| `MC_FUZZY_EN` | Chọn bộ điều khiển attitude gốc (`0`) hoặc phần mở rộng Fuzzy PID (`1`). |
| `MC_FZ_E_MAX`, `MC_FZ_DE_MAX` | Giới hạn chuẩn hóa sai lệch quaternion và tốc độ biến thiên sai lệch. `MC_FZ_DE_MAX` được nhập bằng deg/s rồi đổi sang rad/s trong module. |
| `MC_FZ_R_I/D`, `MC_FZ_P_I/D`, `MC_FZ_Y_I/D` | Ki/Kd cơ sở của Roll/Pitch/Yaw. Kp cơ sở lấy từ các tham số attitude P sẵn có. |
| `MC_FZ_R_DP/DI/DD`, `MC_FZ_P_DP/DI/DD`, `MC_FZ_Y_DP/DI/DD` | Biên độ điều chỉnh Kp/Ki/Kd do suy luận mờ cho từng trục. Mỗi tên có dấu `/` trong bảng là cách viết gộp các tham số, không phải tên nhập trực tiếp vào console. |
| `MC_FZ_I_LIM`, `MC_FZ_I_ZONE`, `MC_FZ_D_LPF` | Giới hạn tích phân, miền tích phân và tần số cắt lọc nhánh D. |

**Mặc định trong mã nguồn:** `MC_FUZZY_EN = 0` - ở đây sử dụng bộ điều khiển P ban đầu; các Ki/Kd cơ sở và biên độ điều chỉnh mờ đều bằng `0`. Chỉ đổi cờ enable chưa tạo ra một cấu hình Fuzzy PID đã được tinh chỉnh. Cần thiết lập và lưu riêng bộ tham số phù hợp trước khi đánh giá.

Sau khi đã cấu hình bộ tham số thử nghiệm trong SITL, lựa chọn phần mở rộng bằng:

```text
param set MC_FUZZY_EN 1
```

Để chuyển sang sử dụng bộ điều khiển Fuzzy PID. chọn lại bộ điều khiển attitude gốc:

```text
param set MC_FUZZY_EN 0
```
**Lưu ý**: Những bước cập nhật tham số này cần phải được thực hiện khi Drone đang ở vị trí đứng yên chưa cất cánh.
Quá trình so sánh chất lượng các bộ điều khiển thông qua dữ liệu chuyến bay thu nhật được vào thẻ SD có sẵn trên Flight Control. Khi so sánh, cần cùng điều kiện đầu quỹ đạo đặt các tham số khác ngoài bộ điều khiển. Phân tích các file log thu được rồi so sánh.

### 7.5. Build firmware cho MicoAir H743 V2

Để build firmware và nạp lên mạch. Chọn target tương ứng với board trong báo cáo:

```bash
make micoair_h743-v2_default
```

Tệp firmware được tạo tại:

```text
build/micoair_h743-v2_default/micoair_h743-v2_default.px4
```

Nạp custom firmware qua QGroundControl theo [hướng dẫn nạp firmware PX4](https://docs.px4.io/main/en/config/firmware.html), sau đó cấu hình airframe, cảm biến, cơ cấu chấp hành và parameters phù hợp với phần cứng. Thực hiện kiểm tra trên bàn khi tháo cánh quạt; đánh giá trong SITL trước khi thử nghiệm bay thật.


## 8. Hướng hoàn thiện

- Giảm rung cơ khí và tối ưu lọc tín hiệu IMU dựa trên phân tích phổ.
- Tinh chỉnh miền sai lệch nhỏ, các hệ số điều khiển và lọc nhánh D.
- Bổ sung bộ parameters, dữ liệu ULog và kịch bản thử nghiệm có thể tái lập.
- Định lượng chất lượng bám, độ quá điều chỉnh và thời gian xác lập trên các kịch bản đối chứng.

## 9. Tác giả, nền tảng và giấy phép

**Tác giả đồ án/phần mở rộng Fuzzy PID:** Lê Viết Khoa ([khoa-asta](https://github.com/khoa-asta)).  
**Giảng viên hướng dẫn:** ThS. Chu Đức Việt.  
**Đơn vị:** Khoa Tự động hóa, Trường Điện – Điện tử, Đại học Bách khoa Hà Nội.

Dự án được phát triển từ [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot). Mã nguồn nền tảng PX4 và các đóng góp có sẵn thuộc về PX4 Development Team cùng các tác giả tương ứng. Phạm vi đóng góp cá nhân trong fork này được mô tả tại mục 2 và commit triển khai được liên kết ở trên.

Giấy phép gốc của PX4 là **BSD 3-Clause**; xem toàn văn trong [LICENSE](LICENSE). Giữ nguyên các thông báo bản quyền, điều kiện giấy phép, phần miễn trừ trách nhiệm và thông tin giấy phép của các thành phần đi kèm. README này không thay thế hoặc sửa đổi các điều khoản đó.

 Khi sử dụng nền tảng PX4 trong nghiên cứu, có thể tham khảo [CITATION.cff](CITATION.cff) để trích dẫn dự án gốc.
