#include <IBusBM.h>
#include <ESP32Servo.h>
#include "MPU9250.h"
#include <TinyGPS++.h>
#include "BluetoothSerial.h"

static const int GpsRxPin = 5, GpsTxPin = 18;
static const uint32_t GpsBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;


Servo servo;
int servoPin = 4; // GPIO4
int led = 15; //GPIO15 (output PWM at boot)

IBusBM IBusServo;    // IBus object
MPU9250 mpu;
float magBiasX = 137.10;
float magXMax = -100;
float magXMin = 100;
float magBiasY = 481.89;
float magYMax = -100;
float magYMin = 100;
boolean calibrationDone = false;

BluetoothSerial SerialBT;
uint8_t ble_msg[] = "hello\n";

void redLedOn() {
  digitalWrite(led, HIGH);
}

void redLedOff() {
  digitalWrite(led, LOW);
}

void setup() {
  SerialBT.begin("HMS Fraser"); //Bluetooth device name
  Serial1.begin(GpsBaud, SERIAL_8N1, GpsRxPin, GpsTxPin);
  Serial.begin(115200);
  IBusServo.begin(Serial2);
  Serial.println("Start ESP32 Autopilot");

  servo.setPeriodHertz(50);    // standard 50 hz servo
  servo.attach(servoPin, 1000, 2000);
  pinMode(led, OUTPUT);     //Set led as output

  Wire.begin();
  redLedOn();
  delay(2000);
  mpu.setup(0x68);
  mpu.verbose(true);
 
  mpu.setMagneticDeclination(0); // magnetic north
  mpu.setMagBiasX(magBiasX);
  mpu.setMagBiasY(magBiasY);
//  mpu.calibrateAccelGyro();
//  mpu.calibrateMag();
  mpu.printCalibration();
  redLedOff();
}

int idleCntr = 0;
int loopCntr = 0;

int autoServoDir = 1500;

int CYCLE_MILLIS = 20;
int MAX_IDLE_CNTR_CYCLE = 50;
int IDLE_MIDDLE_POINT = 1500;

float yaw, corr;

void incrementIdleCntr() {
  if (idleCntr < MAX_IDLE_CNTR_CYCLE) {
    idleCntr++;
  }
}

float courseCorrection(float desiredCourse, float yaw) {
  float corr = desiredCourse - yaw;
  if (corr < -180) {
    corr += 360;
  }
  if (corr > 180) {
    corr -= 360;
  }
  return corr;
}

float inputToCourse(int input) {
  return ((float)(input - IDLE_MIDDLE_POINT)) / 1000.0 * 360.0;
}

void resetIdleCntr() {
  idleCntr = 0;
}

void handleIdleBeginingPeriodEvent() {
}

int IDLE_TOLERANCE = 5;


boolean isIdle(int input) {
  if ((input < IDLE_MIDDLE_POINT + IDLE_TOLERANCE) && (input > IDLE_MIDDLE_POINT - IDLE_TOLERANCE)) {
    return true;
  } else {
    return false;
  }
}

float heading(float magX, float magY) {
  float heading = atan2(magY, magX);

  // Correct for when signs are reversed.
  if (heading < 0) {
      heading += 2 * PI;
  }

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI) {
      heading -= 2 * PI;
  }
  return heading * 180 / M_PI;
}



void loop() {

  loopCntr++;
  

  int inputDir = IBusServo.readChannel(0);
  int inputA = IBusServo.readChannel(4);
  int inputB = IBusServo.readChannel(5);
  float desiredCourse = inputToCourse(inputA);

  if (isIdle(inputDir)) {
    if (idleCntr == 0) {
      handleIdleBeginingPeriodEvent();
    }
    incrementIdleCntr();
  } else {
    resetIdleCntr();
  }

  if(idleCntr == MAX_IDLE_CNTR_CYCLE) {
    // process autopilot mode
    servo.writeMicroseconds(autoServoDir);
    redLedOn();
  } else {
    servo.writeMicroseconds(inputDir);
    redLedOff();
  }

  if (inputB > 1500) {
    calibrationDone = false;
    mpu.update();
    float biasedMagX = mpu.getMagX() + magBiasX;
    float biasedMagY = mpu.getMagY() + magBiasY;
    
    if (biasedMagX < magXMin) {
      magXMin = biasedMagX;
    }
    if (biasedMagX > magXMax) {
      magXMax = biasedMagX;
    }
    if (biasedMagY < magYMin) {
      magYMin = biasedMagY;
    }
    if (biasedMagY > magYMax) {
      magYMax = biasedMagY;
    }
  } else if (!calibrationDone) {
    magBiasX = (magXMax + magXMin) / 2;
    magBiasY = (magYMax + magYMin) / 2;
    mpu.setMagBiasX(magBiasX);
    mpu.setMagBiasY(magBiasY);
    mpu.printCalibration();
    magXMin = 100;
    magXMax = -100;
    magYMin = 100;
    magYMax = -100;
    calibrationDone = true;
    redLedOn();
    delay(300);
    redLedOff();
    delay(300);
    redLedOn();
    delay(300);
    redLedOff();
  }

  if (loopCntr % 50 == 0) {
    mpu.update();
    yaw = heading(mpu.getMagX(), mpu.getMagY());
    corr = courseCorrection(desiredCourse, yaw);
    autoServoDir = (int) (1500 - corr * 20);
    if (autoServoDir < 1000) {
      autoServoDir = 1000;
    }
    if (autoServoDir > 2000) {
      autoServoDir = 2000;
    }
  }

  if (loopCntr % 50 == 0) {

    Serial.print("inputDir : ");
    Serial.print(inputDir, HEX);
    Serial.print(" - CAP : ");
    Serial.print(desiredCourse);
    Serial.print(" - Heading : ");
    Serial.print(yaw);
    Serial.print(" - Corr : ");
    Serial.print(corr);
    Serial.print(" - Cmd : ");
    Serial.print(autoServoDir);
    Serial.print("MagX : ");
    Serial.print(mpu.getMagX());
    Serial.print(" - MagY : ");
    Serial.print(mpu.getMagY());
    Serial.print(" - MagZ : ");
    Serial.print(mpu.getMagZ());

    Serial.println();
  };

  if (SerialBT.available()) {
    while(SerialBT.available()) {
      int incomingByte = SerialBT.read();
      // say what you got:
      Serial.print("I received: ");
      Serial.println(incomingByte, DEC);
      if (incomingByte == 48) { // Command 0
        SerialBT.print("CAP: ");
        SerialBT.println(desiredCourse, 0);
        SerialBT.print("Heading: ");
        SerialBT.println(yaw, 0);
        
      }
    }
  }
  delay(CYCLE_MILLIS);
  
}
