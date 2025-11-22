#ifdef ARDUINO_ENC_COUNTER
// Duoi day co the thay doi, nhung nen la chan PORTD;
// Neu khac can sua code o cho khac
#define LEFT_ENC_PIN_A PD2  // chan 2
#define LEFT_ENC_PIN_B PD3  // chan 3

// Duoi day co the thay doi, nhung nen la chan PORTC
#define RIGHT_ENC_PIN_A PC4  // chan A4
#define RIGHT_ENC_PIN_B PC5  // chan A5
#endif

// Ham doc gia tri encoder
long readEncoder(int i);
// Ham reset mot encoder
void resetEncoder(int i);
// Ham reset ca hai encoder
void resetEncoders();