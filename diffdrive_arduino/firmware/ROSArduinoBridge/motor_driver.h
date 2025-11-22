#ifdef L298_MOTOR_DRIVER
  // Định nghĩa chân BTS7960 cho 2 động cơ:
  // Động cơ phải
  #define RIGHT_RPWM_PIN     5   // Chân PWM thuận động cơ phải
  #define RIGHT_LPWM_PIN     9   // Chân PWM ngược động cơ phải
  // Động cơ trái
  #define LEFT_RPWM_PIN      6   // Chân PWM thuận động cơ trái
  #define LEFT_LPWM_PIN     10   // Chân PWM ngược động cơ trái
  // Nếu dùng chân ENABLE (EN), có thể thêm:
  #define RIGHT_EN_PIN   12
  #define LEFT_EN_PIN    13
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);