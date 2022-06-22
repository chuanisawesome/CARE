/* code written by Chuan-Li Chang and heavely influenced by microcontrollerslab*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGPS++.h>


// The TinyGPS++ object
TinyGPSPlus gps;

Adafruit_MPU6050 mpu;

// THIS IS FOR BUTTON
#include <Arduino.h>
#include <OneButton.h>

#define RXD2 17
#define TXD2 2

//#define BUTTON_PIN 12 // ESP32 GIOP12 pin connected to button's pin
#define BUTTON_PIN 14 // ESP32 GIOP12 pin connected to button's pin
#define BUZZER_PIN 13 // ESP32 GIOP13 pin connected to buzzer's pin
#define MOTOR_PIN 33 // ESP32 GPIO33 pin connected to vibrator motor pin


// The ESP32 has 16 channels which can generate 16 independent waveforms
// We'll just choose PWM channel 0 here
const int TONE_PWM_CHANNEL = 0;

class Button {
  private:
    OneButton button;
    int value;
  public:
    explicit Button(uint8_t pin): button(pin) {
      button.attachClick([](void *scope) {
        ((Button *) scope)->Clicked();
      }, this);
      button.attachLongPressStart([](void *scope) {
        ((Button *) scope)->LongPressed();
      }, this);
    }

    void Clicked() {
      ledcWrite(0, 0);
      digitalWrite(MOTOR_PIN, LOW);
      Serial.println("short-press-DO NOT SEND NOTIFICATIONS");
      value++;
    }

    void LongPressed() {
      ledcWriteNote(TONE_PWM_CHANNEL, NOTE_C, 4);
      digitalWrite(MOTOR_PIN, HIGH);
      Serial.println("LongPress");
    }

    void handle() {
      button.tick();
    }
};

Button button(BUTTON_PIN);

const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

//int data[STORE_SIZE][5]; //array for saving past data
//byte currentIndex=0; //stores current data array index (0-255)
boolean fall = false; //stores if a fall has occurred
boolean trigger1 = false; //stores if first trigger (lower threshold) has occurred
boolean trigger2 = false; //stores if second trigger (upper threshold) has occurred
boolean trigger3 = false; //stores if third trigger (orientation change) has occurred

byte trigger1count = 0; //stores the counts past since trigger 1 was set true
byte trigger2count = 0; //stores the counts past since trigger 2 was set true
byte trigger3count = 0; //stores the counts past since trigger 3 was set true
int angleChange = 0;


void setup() {
  Serial.println(F("Starting..."));
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(115200);

  pinMode(MOTOR_PIN, OUTPUT);
  ledcAttachPin(BUZZER_PIN, TONE_PWM_CHANNEL);
}
void loop() {
  button.handle();

  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
  
  mpu_read();
  //2050, 77, 1947 are values for calibration of accelerometer
  // values may be different for you
  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;

  //270, 351, 136 for gyroscope
  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;

  // calculating Amplitute vactor for 3 axis
  float Raw_AM = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  int AM = Raw_AM * 10;  // as values are within 0 to 1, I multiplied
  // it by for using if else conditions

  Serial.println(AM);
  //Serial.println(PM);
  //delay(500);

  if (AM <= 2 && trigger2 == false) { //if AM breaks lower threshold (0.4g)
    trigger1 = true;
    Serial.println("TRIGGER 1 ACTIVATED");
  }

  if (trigger1 == true) {
    trigger1count++;
    if (AM >= 12) { //if AM breaks upper threshold (3g)
      trigger2 = true;
      Serial.println("TRIGGER 2 ACTIVATED");
      trigger1 = false; trigger1count = 0;
    }
  }
  if (trigger2 == true) {
    trigger2count++;
    angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5); Serial.println(angleChange);
    if (angleChange >= 30 && angleChange <= 400) { //if orientation changes by between 80-100 degrees
      trigger3 = true; trigger2 = false; trigger2count = 0;
      Serial.println(angleChange);
      Serial.println("TRIGGER 3 ACTIVATED");
    }
  }

  if (trigger3 == true) {
    trigger3count++;
    if (trigger3count >= 10) {
      angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
      //delay(10);
      Serial.println(angleChange);
      if ((angleChange >= 0) && (angleChange <= 10)) { //if orientation changes remains between 0-10 degrees
        fall = true; trigger3 = false; trigger3count = 0;
        Serial.println(angleChange);
      }
      else
      { //user regained normal orientation
        trigger3 = false; trigger3count = 0;
        Serial.println("TRIGGER 3 DEACTIVATED");
      }
    }
  }

  if (fall == true)
  { //in event of a fall detection
    Serial.println("FALL DETECTED");
    ledcWriteNote(TONE_PWM_CHANNEL, NOTE_C, 4);
    digitalWrite(MOTOR_PIN, HIGH);
    fall = false;
  }

  if (trigger2count >= 6)
  { //allow 0.5s for orientation change
    trigger2 = false; trigger2count = 0;
    Serial.println("TRIGGER 2 DECACTIVATED");
  }

  if (trigger1count >= 6)
  { //allow 0.5s for AM to break upper threshold
    trigger1 = false; trigger1count = 0;
    Serial.println("TRIGGER 1 DECACTIVATED");
  }
  delay(100);
}

void mpu_read() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

// GPS PRINT TO SERIAL MONITOR
void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
