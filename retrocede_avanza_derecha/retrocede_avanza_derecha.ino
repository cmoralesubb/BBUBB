int tiempo = 2000;

int velocidad_baja = 170;
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
  Serial1.begin(38400);


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
  char c = Serial1.read();

  if (c == 'g') {
    digitalWrite (IN1, LOW);
    digitalWrite (IN2, LOW);

    digitalWrite (IN3, LOW);
    digitalWrite (IN4, LOW);
  }

  //Motor trasero sigue su rumbo
  if (c == 'a') {
    
    // motor delantero a un lado
    analogWrite(ENA, velocidad_maxima);
    digitalWrite (IN1, LOW);
    digitalWrite (IN2, HIGH);
    
    // motor trasero atras
    analogWrite(ENB, velocidad_maxima);
    digitalWrite (IN3, HIGH);
    digitalWrite (IN4, LOW);
    delay(2000);

    //Dobla al sentido contrario hacia delante
    digitalWrite (IN1, HIGH);
    digitalWrite (IN2, LOW);
    
    digitalWrite (IN3, LOW);
    digitalWrite (IN4, HIGH);
    
    delay(2000);


    digitalWrite (IN1, LOW);
    digitalWrite (IN2, HIGH);
    delayMicroseconds(9000);

    digitalWrite (IN1, LOW);
    digitalWrite (IN2, LOW);

    delay(2000);

  }

}


