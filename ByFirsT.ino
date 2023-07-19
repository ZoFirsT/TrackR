// รวมไลบรารีที่จำเป็นสำหรับควบคุม PID
#include <PID_v1.h>

// ค่าคงที่สำหรับขาควบคุมมอเตอร์
const int leftMotorPWM = 9;
const int leftMotorDir = 8;
const int rightMotorPWM = 10;
const int rightMotorDir = 11;

// ค่าคงที่สำหรับขาเซ็นเซอร์
const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// ตัวแปรสำหรับ PID control
double kp = 1.0;  // ค่าคงที่สัมพัทธ์
double ki = 0.0;  // ค่าคงที่ปริภูมิ
double kd = 0.0;  // ค่าคงที่เส้นผ่านแหล่งแปลง
double setpoint = 3.5;  // ตำแหน่งเป้าหมายของเส้น (หมายเลขของเซ็นเซอร์)
double input, output, target;

// กำหนด PID controller
PID linePID(&input, &output, &target, kp, ki, kd, DIRECT);

// ฟังก์ชันสำหรับอ่านค่าเซ็นเซอร์
void readSensors(int sensorValues[8]) {
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
  }
}

// ฟังก์ชันสำหรับควบคุมมอเตอร์
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  analogWrite(leftMotorPWM, leftSpeed);
  analogWrite(rightMotorPWM, rightSpeed);

  if (leftSpeed >= 0) {
    digitalWrite(leftMotorDir, HIGH);  // ถอยหลัง
  } else {
    digitalWrite(leftMotorDir, LOW);   // หน้า
  }

  if (rightSpeed >= 0) {
    digitalWrite(rightMotorDir, HIGH); // ถอยหลัง
  } else {
    digitalWrite(rightMotorDir, LOW);  // หน้า
  }
}

void setup() {
  // กำหนดขามอเตอร์เป็นเอาต์พุต
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorDir, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorDir, OUTPUT);

  // กำหนดขาเซ็นเซอร์เป็นอินพุต
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // กำหนดค่าเริ่มต้นให้กับ PID
  input = 3.5; // ตำแหน่งเริ่มต้น (กลางเส้น)
  target = setpoint; // ค่าเป้าหมายสำหรับ PID controller
  linePID.SetMode(AUTOMATIC);
  linePID.SetOutputLimits(-255, 255);
}

void loop() {
  // อ่านค่าเซ็นเซอร์
  int sensorValues[8];
  readSensors(sensorValues);

  // คำนวณค่าความคลาดเคลื่อน (ระยะห่างจากกลางเส้น)
  int totalSensors = 0;
  double weightedSum = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] == LOW) {
      totalSensors++;
      weightedSum += i;
    }
  }
  if (totalSensors > 0) {
    input = weightedSum / totalSensors;
  } else {
    // หากไม่ตรวจพบเส้น หยุดมอเตอร์
    setMotorSpeeds(0, 0);
  }

  // คำนวณผลลัพธ์ PID
  linePID.Compute();

  // ปรับความเร็วของมอเตอร์ตามผลลัพธ์ PID
  int baseSpeed = 150; // คุณสามารถปรับความเร็วหลักตามต้องการ
  int leftSpeed = baseSpeed + output;
  int rightSpeed = baseSpeed - output;

  // จำกัดความเร็วของมอเตอร์เพื่อป้องกันความไม่เสถียร
  if (leftSpeed > 255) leftSpeed = 255;
  if (leftSpeed < -255) leftSpeed = -255;
  if (rightSpeed > 255) rightSpeed = 255;
  if (rightSpeed < -255) rightSpeed = -255;

  // ปรับความเร็วมอเตอร์
  setMotorSpeeds(leftSpeed, rightSpeed);
}
