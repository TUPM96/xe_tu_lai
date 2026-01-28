/*
 * Arduino Code cho Xe Tự Lái - Điều khiển Ackermann Steering
 * 
 * Nhận lệnh từ Raspberry Pi qua Serial và điều khiển:
 * - Servo cho bánh lái (Ackermann steering)
 * - 1 Motor DC chính cho tốc độ tiến/lùi
 * 
 * Giao thức Serial:
 * - "V:linear:angular\n"  : Điều khiển (linear m/s, angular rad/s)
 * - "S:angle\n"           : Debug - set servo trực tiếp (0-180 độ) để chỉnh/đẩy góc
 * - "C:angle\n"           : Set góc mặc định (center) - angle là độ, dùng làm "thẳng" từ giờ
 * - "M:max_speed:min_pwm\n": Set tốc độ tối đa (m/s) và PWM tối thiểu (0-255)
 *
 * Ví dụ: "S:88\n" -> servo quay tới 88 độ. "C:88\n" -> đặt 88 làm góc mặc định.
 * Ví dụ: "M:0.3:100\n" -> max speed = 0.3 m/s, min PWM = 100
 */

// ==================== CẤU HÌNH PIN ====================
// Servo bánh lái
#define SERVO_PIN 9

// Motor Driver (L298N hoặc TB6612) - 1 motor DC chính
// Motor chính (điều khiển tốc độ tiến/lùi)
#define MOTOR_IN1 2
#define MOTOR_IN2 3
#define MOTOR_PWM 5

// ==================== THAM SỐ ROBOT ====================
// Kích thước robot (phải khớp với URDF)
const float WHEELBASE = 0.4;      // Khoảng cách giữa bánh trước và sau (m)
const float TRACK_WIDTH = 0.21;   // Khoảng cách giữa 2 bánh trái/phải (m)
const float MAX_STEER_ANGLE = 0.5236;  // Góc quay tối đa (rad) ~30 độ
const float WHEEL_RADIUS = 0.034; // Bán kính bánh xe (m)

// Tham số servo
const int SERVO_CENTER_DEFAULT = 100;  // Góc giữa mặc định (độ) - co the doi bang lenh C:
int servo_center = SERVO_CENTER_DEFAULT;  // Góc mặc định hien tai (co the set bang "C:angle")
const int SERVO_RANGE = 45;              // ± độ so voi center

// Tham số motor PWM (có thể thay đổi từ Python)
int motor_min_pwm = 153;  // 60% tốc độ tối thiểu (có thể set từ Python)
const int MOTOR_MAX_PWM = 255;
float max_linear_velocity = 1.0;  // Tốc độ tối đa (m/s) - có thể set từ Python

// ==================== BIẾN TOÀN CỤC ====================
#include <Servo.h>
#include <math.h>

Servo steering_servo;

// Biến lưu giá trị hiện tại
float current_linear = 0.0;
float current_angular = 0.0;
unsigned long last_command_time = 0;
const unsigned long COMMAND_TIMEOUT = 500;  // Timeout 500ms nếu không nhận lệnh

// Debug servo: S:angle giữ servo tại góc, không cho cmd_vel ghi đè. -1 = tắt giữ.
int debug_servo_angle = -1;

// Buffer để nhận dữ liệu Serial
String inputString = "";
boolean stringComplete = false;

// ==================== TRẠNG THÁI RẼ CỐ ĐỊNH ====================
// Khi ROS gửi angular đủ lớn, Arduino tự thực hiện kịch bản rẽ:
// - Rẽ phải: servo = 130 độ, chạy ~0.5m, rồi trả về center và chạy thẳng tiếp
// - Rẽ trái: servo = 70 độ, chạy ~0.5m, rồi trả về center và chạy thẳng tiếp
enum TurnState { TURN_IDLE, TURN_RUNNING };
TurnState turn_state = TURN_IDLE;
int turn_direction = 0;                 // 1 = phải, -1 = trái
unsigned long turn_start_time = 0;

const float TURN_DISTANCE_M      = 0.5f;   // 50 cm
const float TURN_SPEED_MPS       = 0.20f;  // tốc độ khi rẽ (m/s)
const float STRAIGHT_SPEED_MPS   = 0.20f;  // tốc độ sau khi rẽ xong
const float TURN_TRIGGER_ANGULAR = 0.30f;  // |angular| lớn hơn ngưỡng này sẽ kích hoạt chế độ rẽ

const int TURN_SERVO_RIGHT_ANGLE = 130;    // góc servo khi rẽ phải
const int TURN_SERVO_LEFT_ANGLE  = 70;     // góc servo khi rẽ trái

// ==================== SETUP ====================
void setup() {
  // Khởi tạo Serial với baudrate 115200
  Serial.begin(115200);
  Serial.setTimeout(50);  // Timeout 50ms
  
  // Khởi tạo servo
  steering_servo.attach(SERVO_PIN);
  steering_servo.write(servo_center);  // Đặt servo về vị trí giữa (góc mặc định)
  delay(100);
  
  // Khởi tạo motor pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  
  // Dừng motor ban đầu
  stopMotors();
  
  // Reserve buffer cho Serial
  inputString.reserve(50);
  
  // Chờ Serial sẵn sàng
  delay(500);
  
  // Gửi tín hiệu sẵn sàng
  Serial.println("READY");
  
  // Debug
  Serial.println("Arduino Ackermann Motor Control Initialized");
  Serial.print("Wheelbase: "); Serial.println(WHEELBASE);
  Serial.print("Track Width: "); Serial.println(TRACK_WIDTH);
  Serial.print("Max Steer Angle: "); Serial.println(MAX_STEER_ANGLE);
}

// ==================== LOOP CHÍNH ====================
void loop() {
  // Đọc dữ liệu từ Serial
  readSerial();
  
  // Kiểm tra timeout
  // Chỉ dừng motor nếu timeout và vẫn đang chạy
  if (millis() - last_command_time > COMMAND_TIMEOUT) {
    // Timeout: dừng motor nếu đang chạy
    if (abs(current_linear) > 0.01 || abs(current_angular) > 0.01) {
      stopMotors();
      // Reset chỉ một lần để tránh reset liên tục
      static unsigned long last_timeout_reset = 0;
      if (millis() - last_timeout_reset > 100) {  // Chỉ reset mỗi 100ms
        current_linear = 0.0;
        current_angular = 0.0;
        last_timeout_reset = millis();
      }
    }
  }
  
  // Cập nhật điều khiển
  updateControl();
  
  // Delay nhỏ để ổn định
  delay(10);  // 100Hz update rate
}

// ==================== ĐỌC SERIAL ====================
void readSerial() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        parseCommand(inputString);
        inputString = "";
      }
    } else {
      inputString += inChar;
    }
  }
}

// ==================== PARSE COMMAND ====================
void parseCommand(String cmd) {
  // Format: "V:linear:angular"
  if (cmd.startsWith("V:")) {
    cmd = cmd.substring(2);  // Bỏ "V:"
    
    int firstColon = cmd.indexOf(':');
    if (firstColon > 0) {
      String linearStr = cmd.substring(0, firstColon);
      String angularStr = cmd.substring(firstColon + 1);
      
      current_linear = linearStr.toFloat();
      current_angular = angularStr.toFloat();
      
      // Giới hạn giá trị theo max_linear_velocity (có thể set từ Python)
      current_linear = constrain(current_linear, -max_linear_velocity, max_linear_velocity);
      current_angular = constrain(current_angular, -1.0, 1.0);
      
      last_command_time = millis();
      debug_servo_angle = -1;  // Khi co cmd_vel thi tat che do giu debug

      // Nếu không ở trong chế độ rẽ cố định, kiểm tra xem có cần kích hoạt rẽ không
      if (turn_state == TURN_IDLE) {
        if (fabs(current_linear) > 0.01f && fabs(current_angular) > TURN_TRIGGER_ANGULAR) {
          // Bắt đầu pha rẽ cố định
          turn_state = TURN_RUNNING;
          turn_direction = (current_angular > 0.0f) ? 1 : -1;  // 1: phải, -1: trái
          turn_start_time = millis();
          // Khóa vận tốc rẽ độc lập với lệnh sau
          current_linear  = TURN_SPEED_MPS;
          current_angular = 0.0f;  // không dùng angular trong pha rẽ, chỉ dùng servo cố định
        }
      }
    }
  }
  // Debug: "S:angle" - set servo truc tiep va GIU tai do (cmd_vel khong ghi de)
  else if (cmd.startsWith("S:")) {
    int angle = cmd.substring(2).toInt();
    debug_servo_angle = constrain(angle, 0, 180);
    steering_servo.write(debug_servo_angle);
    Serial.print("Servo set (giu): ");
    Serial.println(debug_servo_angle);
  }
  // Set góc mặc định: "C:angle" - angle la do, dung lam "thang", tat giu debug
  else if (cmd.startsWith("C:")) {
    int angle = cmd.substring(2).toInt();
    servo_center = constrain(angle, 0, 180);
    debug_servo_angle = -1;  // Tat giu, tu gio dung servo_center
    steering_servo.write(servo_center);
    Serial.print("Servo center (mac dinh) = ");
    Serial.println(servo_center);
  }
  // Set tốc độ tối đa và PWM tối thiểu: "M:max_speed:min_pwm"
  else if (cmd.startsWith("M:")) {
    cmd = cmd.substring(2);  // Bỏ "M:"
    int colonIdx = cmd.indexOf(':');
    if (colonIdx > 0) {
      float new_max_speed = cmd.substring(0, colonIdx).toFloat();
      int new_min_pwm = cmd.substring(colonIdx + 1).toInt();

      if (new_max_speed > 0.0 && new_max_speed <= 5.0) {
        max_linear_velocity = new_max_speed;
      }
      if (new_min_pwm >= 0 && new_min_pwm <= 255) {
        motor_min_pwm = new_min_pwm;
      }

      Serial.print("Motor config: max_speed=");
      Serial.print(max_linear_velocity);
      Serial.print(" m/s, min_pwm=");
      Serial.println(motor_min_pwm);
    }
  }
}

// ==================== CẬP NHẬT ĐIỀU KHIỂN ====================
void updateControl() {
  // Nếu đang trong chế độ rẽ cố định, xử lý riêng
  if (turn_state == TURN_RUNNING) {
    handleTurnState();
    return;
  }

  // Nếu đang ở chế độ debug servo (S:angle) thì chỉ giữ servo tại góc đặt
  if (debug_servo_angle >= 0) {
    steering_servo.write(debug_servo_angle);
  } else {
    float steering_angle = calculateSteeringAngle(current_angular);
    controlSteering(steering_angle);
  }
  
  // Điều khiển motor theo current_linear (tiến/lùi)
  controlMotors(current_linear);
}

// ==================== XỬ LÝ PHA RẼ CỐ ĐỊNH ====================
void handleTurnState() {
  unsigned long now = millis();
  unsigned long duration_ms = (unsigned long)((TURN_DISTANCE_M / TURN_SPEED_MPS) * 1000.0f);

  if (now - turn_start_time >= duration_ms) {
    // Hoàn thành pha rẽ: trả servo về giữa, tiếp tục đi thẳng
    steering_servo.write(servo_center);
    current_linear  = STRAIGHT_SPEED_MPS;
    current_angular = 0.0f;
    controlMotors(current_linear);
    turn_state = TURN_IDLE;
    return;
  }

  // Đang rẽ: giữ servo ở góc cố định và chạy với tốc độ TURN_SPEED_MPS
  int target_angle = (turn_direction > 0) ? TURN_SERVO_RIGHT_ANGLE : TURN_SERVO_LEFT_ANGLE;
  steering_servo.write(target_angle);
  controlMotors(TURN_SPEED_MPS);
}

// ==================== TÍNH GÓC LÁI (ACKERMANN) ====================
float calculateSteeringAngle(float angular_velocity) {
  // Nếu angular_velocity = 0, không quay
  if (abs(angular_velocity) < 0.01) {
    return 0.0;
  }
  
  // Công thức Ackermann steering:
  // angular_velocity (rad/s) = linear_velocity / radius
  // -> radius = linear_velocity / angular_velocity
  // steering_angle = atan(wheelbase / radius)
  // -> steering_angle = atan(wheelbase * angular_velocity / linear_velocity)
  // 
  // Lưu ý: Khi lùi (linear < 0), dấu của angular phải đảo ngược để quay đúng hướng
  
  float linear_vel = abs(current_linear);
  if (linear_vel < 0.05) {
    // Tốc độ quá nhỏ, với Ackermann không thể quay khi đứng yên
    // Đặt servo về vị trí giữa (sẵn sàng quay khi có tốc độ)
    return 0.0;
  }
  
  // Tính góc quay theo công thức Ackermann
  // Khi lùi (current_linear < 0), phải đảo dấu angular để quay đúng
  float effective_angular = (current_linear < 0) ? -angular_velocity : angular_velocity;
  float steer_angle = atan(WHEELBASE * effective_angular / linear_vel);
  
  // Giới hạn góc quay
  steer_angle = constrain(steer_angle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
  
  return steer_angle;
}

// ==================== ĐIỀU KHIỂN SERVO ====================
void controlSteering(float steering_angle_rad) {
  // Chuyển đổi từ radians sang góc servo (0-180 độ)
  // steering_angle: -MAX_STEER_ANGLE đến +MAX_STEER_ANGLE (radians)
  // servo_angle: servo_center ± SERVO_RANGE (degrees)
  
  int servo_min = servo_center - SERVO_RANGE;
  int servo_max = servo_center + SERVO_RANGE;
  
  float normalized = steering_angle_rad / MAX_STEER_ANGLE;  // -1.0 đến 1.0
  int servo_angle = servo_center + (int)(normalized * SERVO_RANGE);
  
  servo_angle = constrain(servo_angle, servo_min, servo_max);
  steering_servo.write(servo_angle);
}

// ==================== ĐIỀU KHIỂN MOTOR ====================
void controlMotors(float linear_velocity) {
  // Chuyển đổi vận tốc tuyến tính (m/s) sang PWM
  // v = ω * r -> ω = v / r
  // PWM tỷ lệ với vận tốc góc bánh xe
  
  if (abs(linear_velocity) < 0.01) {
    // Tốc độ quá nhỏ, dừng motor
    stopMotors();
    return;
  }
  
  // Tính tốc độ góc bánh xe (rad/s)
  float wheel_angular_velocity = linear_velocity / WHEEL_RADIUS;

  // Chuyển đổi sang PWM (motor_min_pwm đến MOTOR_MAX_PWM)
  // Giả sử max angular velocity = max_linear_velocity / WHEEL_RADIUS
  float max_wheel_angular = max_linear_velocity / WHEEL_RADIUS;
  float normalized = abs(wheel_angular_velocity / max_wheel_angular);
  normalized = constrain(normalized, 0.0, 1.0);

  // Map từ motor_min_pwm đến MOTOR_MAX_PWM
  int pwm_value = motor_min_pwm + (int)(normalized * (MOTOR_MAX_PWM - motor_min_pwm));

  // Giới hạn PWM
  pwm_value = constrain(pwm_value, motor_min_pwm, MOTOR_MAX_PWM);
  
  // Xác định hướng quay
  // Lưu ý: Động cơ đang lắp NGƯỢC chiều, nên map:
  //  - linear_velocity > 0  (tiến)  -> setMotorsBackward
  //  - linear_velocity < 0  (lùi)   -> setMotorsForward
  if (linear_velocity > 0) {
    // Lệnh tiến nhưng động cơ lắp ngược -> dùng hàm lùi
    setMotorsBackward(pwm_value);
  } else {
    // Lệnh lùi -> dùng hàm tiến
    setMotorsForward(pwm_value);
  }
}

// ==================== CÁC HÀM ĐIỀU KHIỂN MOTOR ====================
void setMotorsForward(int pwm) {
  // Motor chính - tiến
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, pwm);
}

void setMotorsBackward(int pwm) {
  // Motor chính - lùi
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  analogWrite(MOTOR_PWM, pwm);
}

void stopMotors() {
  // Dừng motor
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);
}

