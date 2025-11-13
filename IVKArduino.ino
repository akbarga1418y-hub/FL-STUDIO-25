#include <Wire.h>
#include <DynamixelSerial.h>
#include <math.h>
#include <VL53L0X.h>

#define ID_TANGAN_1 2
#define ID_TANGAN_2 5
#define ID_TANGAN_3 5
#define ID_TANGAN_4 6
#define ID_TANGAN_5 10
#define ID_TANGAN_6 4
#define ID_TANGAN_8 3
int step = 1;
float pos2 = 348;
float pos3 = 877;
float pos4 = 616;
float pos6 = 512;
// Panjang lengan (cm)
const float L1 = 18.0;
const float L2 = 15.5;
const float L3 = 7.5;
String inputData = "";
float T = 20;
// ------------------- Konfigurasi VL53L0X -------------------
VL53L0X sensor;
// --- variabel sweep ---
bool sweepEnabled = false;   // diaktifkan lewat perfloatah "n"
bool sweeping     = false;   // berjalan jika ex=2000,ey=2000
bool sweepForward = true;
float  sweepPos     = 0;
unsigned long lastStepTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  sensor.setTimeout(500);
  Dynamixel.setSerial(&Serial1);
  Dynamixel.begin(1000000, 2);
}

void loop() {
  //uint16_t ez = sensor.readRangeSingleMillimeters()/10;
  //Serial.println(ez);
  // === Baca data serial non-blocking ===
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(inputData);
      inputData = "";
    } else {
      inputData += c;
    }
  }

  // === Jalankan sweep servo jika aktif ===
  if (sweeping && millis() - lastStepTime > 50) {
    lastStepTime = millis();

    if (sweepForward) {
      sweepPos += 10;
      if (sweepPos >= 1023) { sweepPos = 1023; sweepForward = false; }
    } else {
      sweepPos -= 10;
      if (sweepPos <= 0)   { sweepPos = 0;   sweepForward = true;  }
    }

    Dynamixel.moveSpeed(ID_TANGAN_1, sweepPos, 50);
    Dynamixel.moveSpeed(ID_TANGAN_2, 348, 50);
    Dynamixel.moveSpeed(ID_TANGAN_3, 348, 50);
    Dynamixel.moveSpeed(ID_TANGAN_4, 877, 50);
    Dynamixel.moveSpeed(ID_TANGAN_5, 877, 50);
    Dynamixel.moveSpeed(ID_TANGAN_6, 616, 50);
    Dynamixel.moveSpeed(ID_TANGAN_8, pos6, 50);
  }
}

// ---------------------------
// Fungsi proses command
// ---------------------------
void handleCommand(String cmd) {
  //if (cmd.indexOf(',') > 0) {  
    float idx = cmd.indexOf(',');
    float idx1 = cmd.indexOf(',',idx + 1);
    float ex = cmd.substring(0, idx).toInt();
    float ey = cmd.substring(idx + 1, idx1).toInt();
    float ez = cmd.substring(idx1 + 1).toInt();

    // === cek kondisi trigger 2000,2000 ===
    if (ex <= 640 ) {
      moveToTarget(ex, ey, ez);
    } else {
      // hanya kalau sweeping belum aktif, ikuti target normal
      if (sweepForward) {
      sweepPos += 5;
      if (sweepPos >= 1023) { sweepPos = 1023; sweepForward = false; }
    } else {
      sweepPos -= 5;
      if (sweepPos <= 0)   { sweepPos = 0;   sweepForward = true;  }
    }

    Dynamixel.moveSpeed(ID_TANGAN_1, sweepPos, 50);
    Dynamixel.moveSpeed(ID_TANGAN_2, 348, 50);
    Dynamixel.moveSpeed(ID_TANGAN_3, 348, 50);
    Dynamixel.moveSpeed(ID_TANGAN_4, 877, 50);
    Dynamixel.moveSpeed(ID_TANGAN_5, 877, 50);
    Dynamixel.moveSpeed(ID_TANGAN_6, 616, 50);
    Dynamixel.moveSpeed(ID_TANGAN_8, pos6, 50);
    }
}

// ---------------------------
void moveToTarget(float ex, float ey, float ez) {
  //int ez = sensor.readRangeSingleMillimeters()/10;
  //Serial.println(ez);
  if(ex >= -30 && ex <= 30 && ey >= -30 && ey <= 30) {  
    
    double c = sqrt((T * T) + (ez * ez)); //T = Tinggi, ez = jarak end effector ke objek
    float t11 = acos(((pow(L1,2)) + (pow(c,2)) - (pow(L2,2))) / (2*L1*c));
    float t111 = atan(ez/T) + t11;
    float t211 = acos(((pow(L1,2)) + (pow(L2,2)) - (pow(c,2))) / (2*L1*L2));
    float t311 = (3.14 - t11 - 1.57) + (3.14 - atan(ez/T) - t211);
    // --- Offset untuk servo ---
    int d11 = (degrees((1.57 + (1.57 - t11)) - 0.523) / 300.0) * 1024.0;
    int d21 = (degrees(1.57 - t211 + 4.71 - 0.523) / 300.0) * 1024.0;
    int d31 = (degrees(1.57 - t311 + 4.71 - 0.523) / 300.0) * 1024.0;

    Dynamixel.moveSpeed(ID_TANGAN_2, d11, 100);
    Dynamixel.moveSpeed(ID_TANGAN_3, d11, 100);
    Dynamixel.moveSpeed(ID_TANGAN_4, d21, 100);
    Dynamixel.moveSpeed(ID_TANGAN_5, d21, 100);
    Dynamixel.moveSpeed(ID_TANGAN_6, d31-40, 100);
    Dynamixel.moveSpeed(ID_TANGAN_8, pos6, 100);
    Serial.print("Cutting : ");
    Serial.print(10 * ez * sin(((sweepPos/1024)*300)-60));// Nilai x
    Serial.print(",");
    Serial.print(10 * ez *  cos(((sweepPos/1024)*300)-60));// Nilai y
    Serial.print(",");
    Serial.print(T);// Nilai Z
    Serial.print(",");
    Serial.print((t111/3.14)*180); //joint 1
    Serial.print(",");
    Serial.print((t211/3.14)*180);// joint 2
    Serial.print(",");
    Serial.println((t311/3.14)*180);// joint 3
    delay(4000);
    Dynamixel.moveSpeed(ID_TANGAN_8, 720, 100);//cut
    delay(4000);
  }
  else{
    //Perhitungan error x mengatur jofloat0
    float pos = (ex/320)*9.108;
    sweepPos-=pos;
    Dynamixel.moveSpeed(ID_TANGAN_1, sweepPos, 50);
    //Perhitungan eeror y mengatur jofloat1,2,3
    float e = ((ey/240)*0.225);
    T -= e;
    float t1 = acos(((pow(L1,2)) + (pow(T,2)) - (pow(L2,2))) / (2*L1*T));
    float t2 = acos(((pow(L1,2)) + (pow(L2,2)) - (pow(T,2))) / (2*L1*L2));
    float t3 = (3.14 - t1 - 1.57) + (3.14 - t2);
    Serial.print("Tracing : ");
    Serial.print(((sweepPos/1024)*300)-60); // joint 0
    Serial.print(", ");
    Serial.print((t1/3.14)*180);
    Serial.print(", ");
    Serial.print((t2/3.14)*180);
    Serial.print(", ");
    Serial.println((t3/3.14)*180);

    // --- Offset untuk servo ---
    int d1 = (degrees((1.57 + (1.57 - t1)) - 0.523) / 300.0) * 1024.0;
    int d2 = (degrees(1.57 - t2 + 4.71 - 0.523) / 300.0) * 1024.0;
    int d3 = (degrees(1.57 - t3 + 4.71 - 0.523) / 300.0) * 1024.0;
    Dynamixel.moveSpeed(ID_TANGAN_2, d1, 50);
    Dynamixel.moveSpeed(ID_TANGAN_3, d1, 50);
    Dynamixel.moveSpeed(ID_TANGAN_4, d2, 50);
    Dynamixel.moveSpeed(ID_TANGAN_5, d2, 50);
    Dynamixel.moveSpeed(ID_TANGAN_6, d3-40, 50);
    Dynamixel.moveSpeed(ID_TANGAN_8, pos6, 50);
    if (T >=30){
      T = 30;
    }
    else if (T <=10){
      T = 10;
    }
    if (sweepPos <= 0){
      sweepPos = 0;
    }
    else if (sweepPos >= 1023){
      sweepPos = 1023;
    }
  }
}
