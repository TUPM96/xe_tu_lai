# Bộ Điều Khiển Động Cơ Arduino

Đoạn mã này biến Arduino thành một bộ điều khiển động cơ!
Nó cung cấp giao diện serial đơn giản để giao tiếp với máy tính cấp cao (ví dụ chạy ROS), và tạo ra tín hiệu PWM phù hợp cho mạch điều khiển động cơ, để điều khiển hai động cơ.

Đây là bản fork của mã gốc, với một số thay đổi và đã loại bỏ các node ROS (xem [repo này](https://github.com/joshnewans/serial_motor_demo) để thay thế). Xem `README-orig.md` để biết README gốc.

Tôi chỉ sử dụng một phần chức năng, nên không chắc những phần khác có hoạt động không, ngoài những gì được ghi chú bên dưới.
Rất hoan nghênh phản hồi/cải tiến (nhưng không hứa sẽ phản hồi nhanh). Hiện tôi chỉ dùng driver L298N và chế độ encoder của Arduino.

TODO
- Hoàn thiện README này

## Chức năng

Chức năng chính là nhận yêu cầu tốc độ động cơ qua kết nối serial, và cung cấp phản hồi encoder.
Mã gốc có thêm các tính năng khác - ví dụ đọc/ghi chân digital/analog, điều khiển servo, nhưng tôi chưa từng dùng.

Các lệnh chính cần biết:

- `e` - Động cơ trả về số đếm encoder hiện tại của mỗi động cơ
- `r` - Đặt lại giá trị encoder
- `o <PWM1> <PWM2>` - Đặt tốc độ PWM thô cho mỗi động cơ (-255 đến 255)
- `m <Spd1> <Spd2>` - Đặt tốc độ động cơ theo vòng lặp đóng (đơn vị: số đếm mỗi vòng lặp, mặc định 30 vòng/giây, nên `(số đếm mỗi giây)/30`)
- `p <Kp> <Kd> <Ki> <Ko>` - Cập nhật thông số PID

## Lưu ý

Một số lưu ý nhanh:

- Có tự động timeout (mặc định 2 giây) nên cần gửi lệnh liên tục để động cơ tiếp tục chạy
- Thứ tự thông số PID là PDI (?)
- Tốc độ động cơ tính theo số đếm mỗi vòng lặp
- Baud rate mặc định 57600
- Cần ký tự xuống dòng (CR)
- Đảm bảo đã bật serial (user thuộc nhóm dialout)
- Xem thêm README gốc để biết thêm chi tiết

## TODO (có thể làm)
- Tài liệu hóa cách chỉnh PID
- Đổi đầu vào tốc độ sang số đếm mỗi giây
- Thêm/kiểm tra các loại driver khác
- Thêm/kiểm tra các chức năng khác