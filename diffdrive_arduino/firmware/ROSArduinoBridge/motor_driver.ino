#ifdef USE_BASE

#ifdef POLOLU_VNH5019
  // Them thu vien Pololu VNH5019
  #include "DualVNH5019MotorShield.h"

  // Tao doi tuong dieu khien dong co
  DualVNH5019MotorShield drive;

  // Ham khoi tao driver dong co
  void initMotorController() {
    drive.init();
  }

  // Ham dat toc do cho mot dong co
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // Ham tien loi dat toc do cho ca hai dong co
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined POLOLU_MC33926
  // Them thu vien Pololu MC33926
  #include "DualMC33926MotorShield.h"

  // Tao doi tuong dieu khien dong co
  DualMC33926MotorShield drive;

  // Ham khoi tao driver dong co
  void initMotorController() {
    drive.init();
  }

  // Ham dat toc do cho mot dong co
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // Ham tien loi dat toc do cho ca hai dong co
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined L298_MOTOR_DRIVER
  void initMotorController() {
    pinMode(RIGHT_RPWM_PIN, OUTPUT);
    pinMode(RIGHT_LPWM_PIN, OUTPUT);
    pinMode(RIGHT_EN_PIN, OUTPUT);
    pinMode(LEFT_RPWM_PIN, OUTPUT);
    pinMode(LEFT_LPWM_PIN, OUTPUT);
    pinMode(LEFT_EN_PIN, OUTPUT);

    // Stop ban đầu
    digitalWrite(RIGHT_RPWM_PIN, HIGH);
    digitalWrite(RIGHT_LPWM_PIN, HIGH);
    digitalWrite(LEFT_RPWM_PIN, HIGH);
    digitalWrite(LEFT_LPWM_PIN, HIGH);
    analogWrite(RIGHT_EN_PIN, 0);
    analogWrite(LEFT_EN_PIN, 0);
  }

  void setMotorSpeed(int i, int spd) {
    int pwm = abs(spd);
    if (pwm > 255) pwm = 255;

    if (i == LEFT) {
        if (spd > 0) {
            digitalWrite(LEFT_RPWM_PIN, HIGH);  // Quay thuận
            digitalWrite(LEFT_LPWM_PIN, LOW);
        } else if (spd < 0) {
            digitalWrite(LEFT_RPWM_PIN, LOW);   // Quay ngược
            digitalWrite(LEFT_LPWM_PIN, HIGH);
        } else { // Dừng
            digitalWrite(LEFT_RPWM_PIN, HIGH);
            digitalWrite(LEFT_LPWM_PIN, HIGH);
        }
        analogWrite(LEFT_EN_PIN, pwm);
    } else { // RIGHT
        if (spd > 0) {
            digitalWrite(RIGHT_RPWM_PIN, HIGH); // Quay thuận
            digitalWrite(RIGHT_LPWM_PIN, LOW);
        } else if (spd < 0) {
            digitalWrite(RIGHT_RPWM_PIN, LOW);  // Quay ngược
            digitalWrite(RIGHT_LPWM_PIN, HIGH);
        } else { // Dừng
            digitalWrite(RIGHT_RPWM_PIN, HIGH);
            digitalWrite(RIGHT_LPWM_PIN, HIGH);
        }
        analogWrite(RIGHT_EN_PIN, pwm);
    }
  }

  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
      setMotorSpeed(LEFT, leftSpeed);
      setMotorSpeed(RIGHT, rightSpeed);
  }
#else
  // Bao loi neu chua chon driver dong co
  #error A motor driver must be selected!
#endif

#endif