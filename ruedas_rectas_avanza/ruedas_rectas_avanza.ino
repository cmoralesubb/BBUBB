const int button = ;
int button_state = LOW;
int flag = 0;

 
int tiempo=2000;

int velocidad_baja=170;
int velocidad_media=200;
int velocidad_maxima=255;

byte IN1=9;  
byte IN2=10;

byte IN3=11; 
byte IN4=12;

byte ENA=8; // Motor A Delantero
byte ENB=13; // motor B Trasero




void setup() {
  
  Serial.begin(9600);
  delay(3000);
  pinMode (IN1, OUTPUT);    // Input1 conectada al pin _ 
  pinMode (IN2, OUTPUT);    // Input2 conectada al pin _ 
  pinMode(ENA,OUTPUT);

  /*MOTOR TRASERO*/
  pinMode (IN3, OUTPUT);    // Input3 conectada al pin _ 
  pinMode (IN4, OUTPUT);    // Input4 conectada al pin _
  pinMode(ENB,OUTPUT);
do{
  button_state = digitalRead(button);
  if(button_state == HIGH){
    flag = 1;
  }
}while(flag == 0);
 
}

void loop() {

  
    //Motor trasero sigue su rumbo

      //rueda delantera recta
        analogWrite(ENA,velocidad_maxima);
        digitalWrite (IN1,LOW);
        digitalWrite (IN2,LOW);
      //rueda trasera avanza
        analogWrite(ENB,velocidad_maxima);
        digitalWrite (IN3,LOW);
        digitalWrite (IN4,HIGH);
   
    
}

