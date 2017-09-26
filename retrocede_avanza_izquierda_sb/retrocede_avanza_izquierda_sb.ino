int tiempo = 2000;
int i;
int velocidad_baja = 170;
int velocidad_media = 200;
int velocidad_maxima = 200;

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

  //Motor trasero sigue su rumbo
 
    //Motor trasero sigue su rumbo
    //Motor delantero a un lado

    digitalWrite (IN1, LOW);
    digitalWrite (IN2, HIGH);
    analogWrite(ENA, velocidad_maxima);
    
    // motor trasero atras
    analogWrite(ENB, velocidad_maxima);
    digitalWrite (IN3, HIGH);
    digitalWrite (IN4, LOW);
    
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

    delay(1000);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
    delay(3000);
    /*
    digitalWrite (IN3, LOW);
    digitalWrite (IN4, LOW);
    */
    


    

}

 
