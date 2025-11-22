Tổng quan
---------
Nhánh này (indigo-devel) dành cho ROS Indigo trở lên và sử dụng hệ thống build Catkin. Có thể cũng tương thích với ROS Hydro.

Bộ stack ROS này bao gồm một thư viện Arduino (tên là ROSArduinoBridge) và một tập hợp các gói ROS để điều khiển robot dựa trên Arduino bằng các message và service chuẩn của ROS. Stack này **không** phụ thuộc vào ROS Serial.

Các tính năng chính:

* Hỗ trợ trực tiếp cảm biến siêu âm Ping và cảm biến hồng ngoại Sharp (GP2D12)
* Có thể đọc dữ liệu từ cảm biến analog và digital thông thường
* Có thể điều khiển output digital (ví dụ bật/tắt công tắc hoặc LED)
* Hỗ trợ servo PWM
* Bộ điều khiển base có thể cấu hình nếu sử dụng đúng phần cứng

Stack này bao gồm một bộ điều khiển base cho robot dẫn động vi sai, nhận message Twist từ ROS và xuất dữ liệu odometry về máy tính. Bộ điều khiển base yêu cầu bộ điều khiển động cơ và encoder để đọc dữ liệu odometry. Phiên bản hiện tại hỗ trợ các phần cứng sau:

* Pololu VNH5019 dual motor controller shield hoặc Pololu MC33926 dual motor shield
* Robogaia Mega Encoder shield hoặc encoder gắn trực tiếp trên bo mạch Arduino
* Driver động cơ L298

Thư viện có thể mở rộng dễ dàng để hỗ trợ các loại driver động cơ và encoder khác.

Tài liệu chính thức ROS
-----------------------
Tài liệu chuẩn theo phong cách ROS có tại:

http://www.ros.org/wiki/ros_arduino_bridge

Yêu cầu hệ thống
----------------
**Python Serial:** Để cài đặt python-serial trên Ubuntu:

    $ sudo apt-get install python-serial

Trên hệ điều hành khác:

    $ sudo pip install --upgrade pyserial

hoặc

    $ sudo easy_install -U pyserial

**Arduino IDE 1.6.6 trở lên:**  
Cần dùng Arduino IDE từ 1.6.6 trở lên để đảm bảo biên dịch firmware đúng. Tải IDE tại https://www.arduino.cc/en/Main/Software.

**Phần cứng:**  
Firmware hoạt động với bất kỳ board Arduino tương thích nào để đọc cảm biến và điều khiển servo PWM. Để dùng bộ điều khiển base, cần có driver động cơ và encoder như trên. Nếu không có, vẫn có thể thử đọc cảm biến và điều khiển servo.

Cài đặt thư viện driver động cơ và encoder phù hợp vào thư mục libraries của Arduino.

Chuẩn bị cổng Serial trên Linux
-------------------------------
Arduino thường kết nối qua /dev/ttyACM# hoặc /dev/ttyUSB#. Xác định bằng:

    $ ls /dev/ttyACM*
    $ ls /dev/ttyUSB*

Đảm bảo bạn có quyền đọc/ghi cổng này. Nếu không, thêm user vào nhóm dialout:

    $ sudo usermod -a -G dialout ten_nguoi_dung

Đăng xuất và đăng nhập lại, kiểm tra bằng:

    $ groups

Cài đặt ros_arduino_bridge
--------------------------
    $ cd ~/catkin_workspace/src
    $ git clone https://github.com/hbrobotics/ros_arduino_bridge.git
    $ cd ~/catkin_workspace
    $ catkin_make

Thư viện Arduino nằm trong gói ros_arduino_firmware.

Để cài đặt ROSArduinoBridge vào Arduino:

    $ cd SKETCHBOOK_PATH
    $ cp -rp `rospack find ros_arduino_firmware`/src/libraries/ROSArduinoBridge ROSArduinoBridge

Nạp firmware ROSArduinoBridge
-----------------------------
* Đảm bảo đã cài thư viện driver động cơ và encoder phù hợp.
* Mở Arduino IDE, vào File->Sketchbook->ROSArduinoBridge để mở sketch.
* Chọn driver động cơ và encoder bằng cách bỏ comment dòng #define tương ứng.
* Nếu muốn điều khiển servo PWM, bỏ comment dòng `#define USE_SERVOS` và chỉnh file servos.h cho đúng số lượng/pin servo.
* Biên dịch và nạp firmware lên Arduino.

Các lệnh firmware
-----------------
Firmware nhận lệnh ký tự đơn qua serial để đọc cảm biến, điều khiển servo, động cơ, encoder.  
**Lưu ý:** Đặt baudrate 57600 và line ending là Carriage return hoặc Both NL & CR trong Serial Monitor.

Danh sách lệnh chính (xem file commands.h):

    a: Đọc analog
    b: Lấy baudrate
    c: Đặt mode chân digital
    d: Đọc digital
    e: Đọc encoder
    m: Đặt tốc độ động cơ
    p: Ping
    r: Reset encoder
    s: Ghi servo
    t: Đọc servo
    u: Cập nhật PID
    w: Ghi digital
    x: Ghi analog

Ví dụ:  
Đọc analog chân 3: `a 3`  
Đặt mode OUTPUT cho chân digital 3: `c 3 1`  
Đọc encoder: `e`  
Chạy động cơ: `m 20 20`

Kiểm tra kết nối động cơ/encoder
-------------------------------
Đặt robot lên giá đỡ, dùng lệnh 'm' để chạy động cơ, 'e' để đọc encoder, 'r' để reset encoder. Đảm bảo bánh xe quay đúng chiều và encoder tăng khi bánh quay tới.

Cấu hình node ros_arduino_python
--------------------------------
Chỉnh file cấu hình trong ros_arduino_python/config, ví dụ:

    port: /dev/ttyUSB0
    baud: 57600
    timeout: 0.1
    rate: 50
    sensorstate_rate: 10
    use_base_controller: False
    base_controller_rate: 10

    # Thông số drivetrain, PID, cảm biến...

**Lưu ý:** Không dùng tab trong file yaml.

Khởi động node ros_arduino_python
---------------------------------
Chỉnh file launch trỏ tới file cấu hình, sau đó chạy:

    $ roslaunch ros_arduino_python arduino.launch

Xem dữ liệu cảm biến:

    $ rostopic echo /arduino/sensor_state
    $ rostopic echo /arduino/sensor/ten_cam_bien

Gửi lệnh Twist và xem odometry:

    $ rostopic pub -1 /cmd_vel geometry_msgs/Twist '{ angular: {z: 0.5} }'
    $ rostopic echo /odom

ROS Services
------------
Gói ros_arduino_python cung cấp một số service ROS như:

    /arduino/digital_set_direction
    /arduino/digital_write
    /arduino/servo_write
    /arduino/servo_read

Dùng encoder onboard (chỉ Arduino Uno)
--------------------------------------
Kết nối encoder như sau:

    Trái A -- pin 2
    Trái B -- pin 3
    Phải A -- pin A4
    Phải B -- pin A5

Chỉnh trong sketch:

    //#define ROBOGAIA
    #define ARDUINO_ENC_COUNTER

Dùng driver L298
----------------
Kết nối chân theo định nghĩa trong motor_driver.h hoặc chỉnh lại cho phù hợp.

Ghi chú
-------
Nếu không có phần cứng base controller, comment dòng `#define USE_BASE` trong sketch và đặt use_base_controller: False trong file cấu hình yaml.