/* Cac ham va kieu du lieu cho dieu khien PID.

   Lay phan lon tu code ArbotiX cua Mike Ferguson:
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* Thong tin setpoint PID cho mot dong co */
typedef struct {
  double TargetTicksPerFrame;    // Toc do muc tieu (so tick moi frame)
  long Encoder;                  // Gia tri encoder hien tai
  long PrevEnc;                  // Gia tri encoder truoc do

  /*
  * Su dung PrevInput thay vi PrevError de tranh derivative kick,
  * Xem giai thich tai: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // Dau vao truoc do
  //int PrevErr;                // Loi truoc do (khong su dung)

  /*
  * Su dung ITerm thay vi Ierror de de dang dieu chinh PID,
  * Xem: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    // Tich phan loi

  long output;                  // Gia tri dieu khien cuoi cung
}
SetPointInfo;

// Khai bao bien PID cho dong co trai va phai
SetPointInfo leftPID, rightPID;

/* Tham so PID */
int Kp = 20;   // He so ti le
int Kd = 12;   // He so dao ham
int Ki = 0;    // He so tich phan
int Ko = 50;   // He so chia PID

unsigned char moving = 0; // Co dang di chuyen khong

/*
* Khoi tao cac bien PID ve 0 de tranh xung dot khi bat dau chay PID
* Dac biet, gan Encoder va PrevEnc bang gia tri encoder hien tai
* Xem: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
*/
void resetPID(){
   leftPID.TargetTicksPerFrame = 0.0;
   leftPID.Encoder = readEncoder(LEFT);
   leftPID.PrevEnc = leftPID.Encoder;
   leftPID.output = 0;
   leftPID.PrevInput = 0;
   leftPID.ITerm = 0;

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoder(RIGHT);
   rightPID.PrevEnc = rightPID.Encoder;
   rightPID.output = 0;
   rightPID.PrevInput = 0;
   rightPID.ITerm = 0;
}

/* Ham PID tinh toan lenh dieu khien moi cho dong co */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //input la so tick encoder tang trong 1 frame
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;

  /*
  * Tranh derivative kick va de dang dieu chinh PID,
  * Xem cac link o tren
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Tich phan loi hoac gioi han output
  // Dung tich phan khi output bi gioi han
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    // Cho phep dieu chinh PID khi chua bi gioi han
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* Doc gia tri encoder va goi ham PID */
void updatePID() {
  // Doc encoder
  leftPID.Encoder = readEncoder(LEFT);
  rightPID.Encoder = readEncoder(RIGHT);

  // Neu khong di chuyen thi khong lam gi
  if (!moving){
    /*
    * Reset PID mot lan de tranh xung dot khi bat dau chay
    * Su dung PrevInput de kiem tra da reset chua
    */
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
    return;
  }

  // Tinh PID cho moi dong co
  doPID(&rightPID);
  doPID(&leftPID);

  // Dat toc do dong co theo PID
  setMotorSpeeds(leftPID.output, rightPID.output);
}