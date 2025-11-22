#ifdef USE_SERVOS

// Ham khoi tao SweepServo
SweepServo::SweepServo()
{
  this->currentPositionDegrees = 0;      // Vi tri hien tai ban dau la 0 do
  this->targetPositionDegrees = 0;       // Vi tri muc tieu ban dau la 0 do
  this->lastSweepCommand = 0;            // Thoi diem quet cuoi la 0
}

// Ham khoi tao servo
void SweepServo::initServo(
    int servoPin,        // Chan ket noi servo
    int stepDelayMs,     // Thoi gian tre moi buoc
    int initPosition)    // Vi tri khoi tao
{
  this->servo.attach(servoPin);                // Gan chan dieu khien servo
  this->stepDelayMs = stepDelayMs;             // Luu thoi gian tre
  this->currentPositionDegrees = initPosition; // Dat vi tri hien tai
  this->targetPositionDegrees = initPosition;  // Dat vi tri muc tieu
  this->lastSweepCommand = millis();           // Luu thoi diem khoi tao
}

// Ham thuc hien quet servo
void SweepServo::doSweep()
{
  // Lay thoi gian da troi qua tu lan quet cuoi
  int delta = millis() - this->lastSweepCommand;

  // Kiem tra neu da den thoi gian thuc hien buoc moi
  if (delta > this->stepDelayMs) {
    // Kiem tra huong di chuyen cua servo
    if (this->targetPositionDegrees > this->currentPositionDegrees) {
      this->currentPositionDegrees++;                // Tang vi tri hien tai
      this->servo.write(this->currentPositionDegrees); // Dieu khien servo
    }
    else if (this->targetPositionDegrees < this->currentPositionDegrees) {
      this->currentPositionDegrees--;                // Giam vi tri hien tai
      this->servo.write(this->currentPositionDegrees); // Dieu khien servo
    }
    // Neu vi tri muc tieu bang vi tri hien tai thi khong lam gi

    // Reset lai thoi gian quet cuoi
    this->lastSweepCommand = millis();
  }
}

// Ham dat vi tri muc tieu moi cho servo
void SweepServo::setTargetPosition(int position)
{
  this->targetPositionDegrees = position;
}

// Ham tra ve doi tuong Servo
Servo SweepServo::getServo()
{
  return this->servo;
}

#endif