#ifndef SERVOS_H
#define SERVOS_H

// So luong servo su dung
#define N_SERVOS 2

// Mang thoi gian tre (ms) giua moi buoc 1 do cua servo
// Tang gia tri nay servo quay cham hon, giam thi quay nhanh hon
// 0 la mac dinh, servo quay nhanh nhat. 150 ms quay rat cham
int stepDelay [N_SERVOS] = { 0, 0 }; // ms

// Mang cac chan ket noi voi servo
byte servoPins [N_SERVOS] = { 3, 4 };

// Mang vi tri khoi tao cua moi servo (tu 0 den 180 do)
byte servoInitPosition [N_SERVOS] = { 90, 90 }; // [0, 180] degrees

// Dinh nghia lop dieu khien servo quet
class SweepServo
{
public:
    SweepServo(); // Ham khoi tao
    void initServo(
        int servoPin,        // Chan ket noi servo
        int stepDelayMs,     // Thoi gian tre moi buoc
        int initPosition);   // Vi tri khoi tao
    void doSweep();         // Ham thuc hien quet servo
    void setTargetPosition(int position); // Dat vi tri muc tieu
    Servo getServo();       // Lay doi tuong Servo

private:
    Servo servo;                    // Doi tuong Servo
    int stepDelayMs;                // Thoi gian tre moi buoc
    int currentPositionDegrees;      // Vi tri hien tai (do)
    int targetPositionDegrees;       // Vi tri muc tieu (do)
    long lastSweepCommand;           // Thoi diem thuc hien lenh quet cuoi
};

// Tao mang doi tuong servo
SweepServo servos [N_SERVOS];

#endif