int tiempo = 2000;

int velocidad_baja = 190;
int velocidad_media = 200;
int velocidad_maxima = 255;

//definicion de variables de los motores
byte IN1 = 9;
byte IN2 = 10;

byte IN3 = 11;
byte IN4 = 12;

byte ENA = 8; // Motor A Delantero
byte ENB = 13; // motor B Trasero



void setup() {

  Serial.begin(9600);

  /*MOTOR DELANTERO*/

  pinMode (IN1, OUTPUT);    // Input1 conectada al pin _
  pinMode (IN2, OUTPUT);    // Input2 conectada al pin _
  pinMode(ENA, OUTPUT);

  /*MOTOR TRASERO*/
  pinMode (IN3, OUTPUT);    // Input3 conectada al pin _
  pinMode (IN4, OUTPUT);    // Input4 conectada al pin _
  pinMode(ENB, OUTPUT);

}

void loop() {
    //Motor delantero a un lado
    analogWrite(ENA, 255);
    digitalWrite (IN1, LOW);
    digitalWrite (IN2, HIGH);
    
    delay(3000);    
   

}

