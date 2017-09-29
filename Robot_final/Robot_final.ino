 /*
    EXPO ROBOTICA UBB 2017
    MATIAS VIDAL GUTIERREZ , CLAUDIO MORALES GUTIERREZ
    IECI
    */

const int LED = 3;
const int button = 4;
int button_state = LOW;
int cont = 50;



//GIROSCOPIO
#include<Wire.h>
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int estado=0;
int a=0;
int b=0;
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

    pinMode (button, INPUT);
    pinMode (LED, OUTPUT);

    do{
    button_state = digitalRead(button);
    if(button_state == HIGH){
      sentido_giro = 1;   // cambiar segun el boton
    }
    else{
      sentido_giro = 0;  // cambiar segun el boton
    }
    delay(100);
    cont = cont +1;
    }while(cont < 40);  
    if(sentido_giro == 1){
      digitalWrite(LED,HIGH);
    }
    else{
      digitalWrite(LED,LOW);
    }

    pista = 1;

    contacto=0;

    activado_inclinacion=0;

}

void loop() {
  Serial.print("Sentido Giro ");
  Serial.println(sentido_giro);
	sensor_izquierdo=digitalRead(sen_izq);
	sensor_derecho=digitalRead(sen_der);   
 
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
        /*
        if((digitalRead(sen_izq) == pista) && (digitalRead(sen_der) == pista) ) { 
        	avanzar();
        } 
        */

      //Detecta el sensor de pistaa derecha
      if ((digitalRead(sen_izq) == pista)  && (digitalRead(sen_der) != pista) ) {
        //Giro a la izquierda
        if(activado_inclinacion==0){
        	emparejar_izquierda();
        	delay(100);   
        } 

    }

      // Detecta la pistaa I zquierda
      if ((digitalRead(sen_izq) != pista) && (digitalRead(sen_der) == pista)) { 
        //Giro a la Derecha 
        if(activado_inclinacion==0){
        	emparejar_derecha();
        	delay(100);   
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
//Captura del gyroscopio
Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
GyY=Wire.read()<<8|Wire.read();
//Serial.println("GyY= ");
//Serial.println(GyY);

			if (GyY>2000){
				estado = -1;
				a = 0;
				b = -1;
      return estado; Serial.println(estado);
  }
  if (GyY<-2000)
  {
  	estado = 1;
  	a = 0;
  	b = 1;
      return estado; Serial.println(estado);
  }
  a = b;
  if (-500 < GyY < 500)
  {
  	estado = 0 + a;
  	b = 0;
      return estado; Serial.println(estado);
  }

}
//Fin captura del gyroscopio



void configuracion_giroscopio(){
Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
}
