#ifdef USE_BASE

#ifdef ROBOGAIA
  // Su dung shield encoder Robogaia Mega
  #include "MegaEncoderCounter.h"

  // Tao doi tuong encoder, khoi tao o che do dem 4X
  MegaEncoderCounter encoders = MegaEncoderCounter(4);

// Ham doc gia tri encoder
long readEncoder(int i) {
  if (i == LEFT) return encoders.YAxisGetCount();
  else return encoders.XAxisGetCount();
}

// Ham reset encoder
void resetEncoder(int i) {
  if (i == LEFT) return encoders.YAxisReset();
  else return encoders.XAxisReset();
}
#elif defined(ARDUINO_ENC_COUNTER)
// Khai bao bien dem encoder ben trai va phai
volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;
// Bang tra cuu trang thai encoder
static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

// Ngat xu ly encoder ben trai, dem xung encoder
ISR (PCINT2_vect){
  static uint8_t enc_last=0;

  enc_last <<=2; // dich bit trang thai cu
  enc_last |= (PIND & (3 << 2)) >> 2; // doc trang thai hien tai vao 2 bit thap nhat

  left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

// Ngat xu ly encoder ben phai, dem xung encoder
ISR (PCINT1_vect){
  static uint8_t enc_last=0;

  enc_last <<=2; // dich bit trang thai cu
  enc_last |= (PINC & (3 << 4)) >> 4; // doc trang thai hien tai vao 2 bit thap nhat

  right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

// Ham doc gia tri encoder
long readEncoder(int i) {
  if (i == LEFT) return left_enc_pos;
  else return right_enc_pos;
}

// Ham reset encoder
void resetEncoder(int i) {
  if (i == LEFT){
    left_enc_pos=0L;
    return;
  } else {
    right_enc_pos=0L;
    return;
  }
}
#else
// Bao loi neu chua chon driver encoder
#error A encoder driver must be selected!
#endif

// Ham reset ca hai encoder
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif