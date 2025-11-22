float microsecondsToCm(long microseconds)
{
  // Toc do am thanh la 340 m/s ~ 29 micro giay tren 1 cm
  // Song ping di va ve, nen lay mot nua quang duong
  return microseconds / 29 / 2;
}

long Ping(int pin) {
  long duration, range;

  // Cam bien PING))) kich hoat bang xung HIGH tu 2 micro giay tro len
  // Tao xung LOW ngan truoc de dam bao xung HIGH sach
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);

  // Su dung cung mot chan de doc tin hieu tra ve tu cam bien PING)))
  // Xung HIGH co do dai la thoi gian tu luc gui den luc nhan duoc phan xa
  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH);

  // Chuyen thoi gian thanh khoang cach (cm)
  range = microsecondsToCm(duration);

  return(range);
}