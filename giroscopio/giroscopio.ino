#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
int estado = 0;
int a = 0;
int b = 0;
//INTERRUPT DETECTION ROUTINE
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void setup(){
  Serial.begin(9600);
  configuracion_giroscopio();

}

void loop(){
  Serial.println(inclinacion());
}

int inclinacion(){

  if (!dmpReady) 
  return;

  while (!mpuInterrupt && fifoCount < packetSize) {

  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    }else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) 
      fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      #ifdef OUTPUT_READABLE_YAWPITCHROLL
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      #endif

      if ((ypr[1] * 180 / M_PI) > 10){
        estado = 1;
        a = 0;
        b = 1;
      return estado; //Serial.println(ypr[1] * 180/M_PI);
    }
    if ((ypr[1] * 180 / M_PI) < -10)
    {
      estado = -1;
      a = 0;
      b = -1;
      return estado; //Serial.println(ypr[1] * 180/M_PI);
    }
    a = b;
    if (-5 < (ypr[1] * 180 / M_PI) < 5)
    {
      estado = 0 + a;
      b = 0;
      return estado; //Serial.println(ypr[1] * 180/M_PI);
    }

  }
}


void configuracion_giroscopio(){
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 Wire.begin();
 TWBR = 24;
 #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
 Fastwire::setup(400, true);
 #endif
 while (!Serial);
 devStatus = mpu.dmpInitialize();
 mpu.setXGyroOffset(220);
 mpu.setYGyroOffset(3);
 mpu.setZGyroOffset(-85);
 mpu.setZAccelOffset(1788); 
 if (devStatus == 0) {

  mpu.setDMPEnabled(true);
  attachInterrupt(0, dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();

  dmpReady = true;

  packetSize = mpu.dmpGetFIFOPacketSize();
}
pinMode(LED_PIN, OUTPUT);
}


