 /*
    EXPO ROBOTICA UBB 2017
    MATIAS VIDAL GUTIERREZ , CLAUDIO MORALES GUTIERREZ
    IECI
    */
//GIROSCOPIO

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
VectorInt16 aaWorld;    // [x, y, z]            meddida del aclerome
VectorFloat gravity;    // [x, y, z]            gravedad
float euler[3];         // [psi, theta, phi]    angulo de euler
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
int estado = 0;
int a = 0;
int b = 0;
//RUTINA DE DETECCION DE LA INTERRUPCION
volatile bool mpuInterrupt = false;    
void dmpDataReady() {
  mpuInterrupt = true;
}
//FIN GIROSCOPIO

  //byte tipo dato de 0-255
  byte sen_izq = 5; // sensor de la izquierda
  byte sen_der = 6; // sensor de la derecha
  byte sen_tras=7;
  
  byte velocidad_maxima = 255;
  byte velocidad_media = 200;
  byte velocidad_minima = 100;
  
  /* INPUTS DE PLACA L298D*/
  
  //Motor delantero A
  byte ENA = 8;
  byte IN1 = 9;
  byte IN2 = 10;
  
  //Motor B Trasero
  byte ENB = 13;
  byte IN3 = 11;
  byte IN4 = 12;
  
  //--SENTIDO DE GIRO--//
  byte sentido_giro;

  byte contacto; //contador de la pista cuando dobla memorizando pista

  int sensor_izquierdo;
  int sensor_derecho;
  int sensor_trasero;
  
  byte inclinacion_actual;

  byte aux_inclinacion;
  byte aux_giro;

  byte activado_inclinacion;

  int pista;

  int tiempo_enderezar = 1000;

  
  void setup() {

    Serial.begin(9600);
    configuracion_giroscopio();



    delay(3000);
    pinMode(sen_izq, INPUT);  // sensor inicia como entrada
    pinMode(sen_der, INPUT);  // snesor inicia como entrada

    /*MOTOR DELANTERO*/
    pinMode(ENA, OUTPUT);
    pinMode (IN1, OUTPUT);    // Input1 conectada al pin _
    pinMode (IN2, OUTPUT);    // Input2 conectada al pin _

    /*MOTOR DELANTERO*/
    pinMode(ENB, OUTPUT);
    pinMode (IN3, OUTPUT);    // Input3 conectada al pin _
    pinMode (IN4, OUTPUT);    // Input4 conectada al pin _

    sentido_giro = 1; // cambiar segun el boton 

    pista = 1;

    contacto=0;

    activado_inclinacion=0;

}

void loop() {
  Serial.println(inclinacion());
  sensor_izquierdo=digitalRead(sen_izq);
  sensor_derecho=digitalRead(sen_der);   

  Serial.print("sensor izquierdo");
  Serial.println(sensor_izquierdo);

  Serial.println("sensor derecho");
  Serial.println(sensor_derecho);


  if(inclinacion()==0){
    analogWrite(ENB, velocidad_media);
    avanzar();
    }else if(inclinacion()==-1){
      analogWrite(ENB, velocidad_maxima);
      activado_inclinacion=1;
      avanzar();
    }

    if ((sensor_izquierdo != pista) || (sensor_derecho != pista)) { 
        //ver el sentido de giro con un boton
        if(activado_inclinacion==1){
          activado_inclinacion=0;
          if (sentido_giro == 1) {
            Serial.println("giro derecha");
            giro_der();
            } else if (sentido_giro == 0) {
              Serial.println("giro izquierda");
              giro_izq();
            }          
          }
        }
}

void detener() {
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);

  digitalWrite (IN3, LOW);
  digitalWrite (IN4, LOW);

}

void avanzar() {
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);

  delay(200);
}

  //trda : tiempo ruedas delanteras atras
  //trta : tiempo ruedas traseras atras
  //trdd : tiempo ruedas delanteras delante
  void giro_der() {
         //Motor trasero sigue su rumbo

    //Motor trasero sigue su rumbo
    //Motor delantero a un lado

    digitalWrite (IN1, HIGH);
    digitalWrite (IN2, LOW);
    analogWrite(ENA, velocidad_maxima);

    // motor trasero atras
    analogWrite(ENB, velocidad_media);
    digitalWrite (IN3, HIGH);
    digitalWrite (IN4, LOW);

    delay(1100);

    //Dobla al sentido contrario hacia delante

    digitalWrite (IN3, LOW);
    digitalWrite (IN4, HIGH);


    digitalWrite (IN1, LOW);
    digitalWrite (IN2, HIGH);

    delay(1200);

    digitalWrite (IN1, HIGH);
    digitalWrite (IN2, LOW);
    delayMicroseconds(8500);

    digitalWrite (IN1, LOW);
    digitalWrite (IN2, LOW);


}

void emparejar_derecha() {
  analogWrite(ENB, velocidad_media);
  analogWrite(ENA,velocidad_maxima);

  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);

  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  delay(500);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  delay(100);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  delayMicroseconds(8500);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  analogWrite(ENB, velocidad_maxima);


}

void giro_izq() {
         //Motor trasero sigue su rumbo

    //Motor trasero sigue su rumbo
    //Motor delantero a un lado

    analogWrite(ENB, velocidad_media);
    digitalWrite (IN3, HIGH);
    digitalWrite (IN4, LOW);


    digitalWrite (IN1, LOW);
    digitalWrite (IN2, HIGH);
    analogWrite(ENA, velocidad_maxima);

    // motor trasero atras

    delay(1100);

    //Dobla al sentido contrario hacia delante


    digitalWrite (IN3, LOW);
    digitalWrite (IN4, HIGH);


    digitalWrite (IN1, HIGH);
    digitalWrite (IN2, LOW);

    delay(1200);

    digitalWrite (IN1, LOW);
    digitalWrite (IN2, HIGH);
    delayMicroseconds(8500);

    digitalWrite (IN1, LOW);
    digitalWrite (IN2, LOW);


}



void emparejar_izquierda() {
  Serial.println("Emparejar_izquierda");
  analogWrite(ENB, velocidad_media);
  analogWrite(ENA,velocidad_maxima);

  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);

  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  delay(500);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  delay(100);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  delayMicroseconds(8500);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  analogWrite(ENB, velocidad_maxima);

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
