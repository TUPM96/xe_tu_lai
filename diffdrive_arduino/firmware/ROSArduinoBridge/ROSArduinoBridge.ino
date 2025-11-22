#define USE_BASE      // Kich hoat code dieu khien base
//#undef USE_BASE     // Khong kich hoat code dieu khien base

/* Khai bao thu vien dieu khien dong co va encoder dang su dung */
#ifdef USE_BASE
   /* Su dung shield Pololu VNH5019 */
   //#define POLOLU_VNH5019

   /* Su dung shield Pololu MC33926 */
   //#define POLOLU_MC33926

   /* Su dung shield encoder RoboGaia */
   //#define ROBOGAIA

   /* Su dung encoder noi truc tiep vao Arduino */
   #define ARDUINO_ENC_COUNTER

   /* Su dung driver dong co L298 */
   #define L298_MOTOR_DRIVER
#endif

//#define USE_SERVOS  // Kich hoat su dung servo PWM nhu dinh nghia trong servos.h
#undef USE_SERVOS     // Khong su dung servo PWM

/* Toc do baud cua cong Serial */
#define BAUDRATE     57600

/* Gia tri PWM toi da */
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Khai bao cac lenh serial */
#include "commands.h"

/* Khai bao cac ham cam bien */
#include "sensors.h"

/* Neu su dung servo thi them thu vien */
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"
#endif

#ifdef USE_BASE
  /* Khai bao ham dieu khien dong co */
  #include "motor_driver.h"

  /* Khai bao ham dieu khien encoder */
  #include "encoder_driver.h"

  /* Khai bao cac tham so va ham PID */
  #include "diff_controller.h"

  /* Vong lap PID chay 30 lan moi giay */
  #define PID_RATE           30     // Hz

  /* Chuyen tan so thanh khoang thoi gian */
  const int PID_INTERVAL = 1000 / PID_RATE;

  /* Bien luu thoi diem tiep theo chay PID */
  unsigned long nextPID = PID_INTERVAL;

  /* Tu dong dung robot neu khong nhan lenh trong khoang thoi gian nay (ms) */
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Khoi tao bien */

// Bien ho tro phan tich lenh serial
int arg = 0;
int index = 0;

// Bien luu ky tu nhap vao
char chr;

// Bien luu lenh don (mot ky tu)
char cmd;

// Mang ky tu luu doi so thu nhat va thu hai
char argv1[16];
char argv2[16];

// Doi so da chuyen thanh so nguyen
long arg1;
long arg2;

/* Xoa cac tham so lenh hien tai */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Thuc hien mot lenh. Cac lenh duoc dinh nghia trong commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK");
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK");
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;
#ifdef USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].getServo().read());
    break;
#endif

#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Dat lai bo dem tu dong dung */
    lastMotorCommand = millis();
    // Print lệnh nhận được dạng 'o speed1 speed2'
    Serial.print("[CMD] o ");
    Serial.print(arg1);
    Serial.print(" ");
    Serial.println(arg2);
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK");
    break;
  case MOTOR_RAW_PWM:
    /* Dat lai bo dem tu dong dung */
    lastMotorCommand = millis();
    resetPID();
    moving = 0; // Tam thoi tat PID
    // Print lệnh nhận được dạng 'p speed1 speed2'
    Serial.print("[CMD] p ");
    Serial.print(arg1);
    Serial.print(" ");
    Serial.println(arg2);
    setMotorSpeeds(arg1, arg2);
    Serial.println("OK");
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Ham setup, chay mot lan khi khoi dong */
void setup() {
  Serial.begin(BAUDRATE);

// Khoi tao dieu khien dong co neu su dung base
#ifdef USE_BASE
  #ifdef ARDUINO_ENC_COUNTER
    // Cai dat cac chan encoder la input
    DDRD &= ~(1<<LEFT_ENC_PIN_A);
    DDRD &= ~(1<<LEFT_ENC_PIN_B);
    DDRC &= ~(1<<RIGHT_ENC_PIN_A);
    DDRC &= ~(1<<RIGHT_ENC_PIN_B);

    // Bat tro keo len cho cac chan encoder
    PORTD |= (1<<LEFT_ENC_PIN_A);
    PORTD |= (1<<LEFT_ENC_PIN_B);
    PORTC |= (1<<RIGHT_ENC_PIN_A);
    PORTC |= (1<<RIGHT_ENC_PIN_B);

    // Cai dat mask ngat cho cac chan encoder ben trai
    PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
    // Cai dat mask ngat cho cac chan encoder ben phai
    PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);

    // Bat ngat PCINT1 va PCINT2 trong mask tong
    PCICR |= (1 << PCIE1) | (1 << PCIE2);
  #endif
  initMotorController();
  resetPID();
#endif

/* Khoi tao servo neu su dung */
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].initServo(
          servoPins[i],
          stepDelay[i],
          servoInitPosition[i]);
    }
  #endif
}

/* Vong lap chinh. Doc va phan tich du lieu tu serial,
   thuc hien lenh hop le. Chay PID va kiem tra tu dong dung.
*/
void loop() {
  while (Serial.available() > 0) {

    // Doc ky tu tiep theo
    chr = Serial.read();

    // Ket thuc lenh bang CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Dung dau cach de phan tach cac phan cua lenh
    else if (chr == ' ') {
      // Chuyen sang doi so tiep theo
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // Doi so dau tien la lenh mot ky tu
        cmd = chr;
      }
      else if (arg == 1) {
        // Doi so co the nhieu ky tu
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

// Neu su dung base, chay PID dinh ky
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // Kiem tra neu vuot qua thoi gian tu dong dung
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0);
    moving = 0;
  }
#endif

// Quet servo
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif
}