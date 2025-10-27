#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
Servo servo1, servo2, servo3, servo4, servo5;

#define servoPin1 16
#define servoPin2 17
#define servoPin3 5
#define servoPin4 18
#define servoPin5 19

#define pirPin 32

float roll, pitch, yaw;
int servoCenter = 90;  // posisi awal servo (tengah)
unsigned long prevTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Inisialisasi sistem...");

  if (!mpu.begin()) {
    Serial.println("MPU6050 gagal diinisialisasi!");
    while (1);
  }

  Serial.println("MPU6050 siap!");

  pinMode(pirPin, INPUT);

  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  servo5.attach(servoPin5);

  // set ke posisi awal
  servo1.write(servoCenter);
  servo2.write(servoCenter);
  servo3.write(servoCenter);
  servo4.write(servoCenter);
  servo5.write(servoCenter);

  prevTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // hitung selisih waktu antar loop
  unsigned long currTime = millis();
  float dt = (currTime - prevTime) / 1000.0; // ubah ke detik
  prevTime = currTime;

  // hitung orientasi dari sensor MPU6050
  roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  yaw += g.gyro.z * dt;  // integrasi kecepatan rotasi untuk dapat sudut yaw

  // baca nilai sensor PIR
  int pirState = digitalRead(pirPin);

  // roll (sumbu Y)
  if (roll > 10) {
    servo1.write(constrain(servoCenter - 90, 0, 180));
    servo2.write(constrain(servoCenter - 90, 0, 180));
  } else if (roll < -10) {
    servo1.write(constrain(servoCenter + 90, 0, 180));
    servo2.write(constrain(servoCenter + 90, 0, 180));
  } else {
    servo1.write(servoCenter);
    servo2.write(servoCenter);
  }

  // pitch (sumbu X)
  if (pitch > 10) {
    servo3.write(constrain(servoCenter + 90, 0, 180));
    servo4.write(constrain(servoCenter + 90, 0, 180));
  } else if (pitch < -10) {
    servo3.write(constrain(servoCenter - 90, 0, 180));
    servo4.write(constrain(servoCenter - 90, 0, 180));
  } else {
    servo3.write(servoCenter);
    servo4.write(servoCenter);
  }

  // yaw
  if (abs(yaw) > 10) { // batas minimal 
    int yawDirection = (yaw > 0) ? 1 : -1;
    servo5.write(constrain(servoCenter + (45 * yawDirection), 0, 180)); 
    delay(1000);
    servo5.write(servoCenter);
  }

  // jika sensor PIR mendeteksi gerakan 
  if (pirState == HIGH) {
    servo1.write(constrain(servoCenter + 45, 0, 180));
    servo2.write(constrain(servoCenter - 45, 0, 180));
    servo3.write(constrain(servoCenter + 90, 0, 180));
    servo4.write(constrain(servoCenter - 90, 0, 180));
    servo5.write(constrain(servoCenter + 20, 0, 180));

    delay(1000);

    // reset posisi
    servo1.write(servoCenter);
    servo2.write(servoCenter);
    servo3.write(servoCenter);
    servo4.write(servoCenter);
    servo5.write(servoCenter);
  }

  // tampilkan data
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print(" | Pitch: "); Serial.print(pitch);
  Serial.print(" | Yaw: "); Serial.println(yaw);

  delay(100);
}
