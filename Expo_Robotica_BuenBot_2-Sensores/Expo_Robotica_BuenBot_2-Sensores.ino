/*
EVALUAR SI LEER ANTES LAS VARIABLES DE DATOS

VER VUELTAS CORTAS DE GIRAR
*/

/*
EXPO ROBOTICA UBB 2017
CLAUDIO MORALES GUTIERREZ
IECI
*/
int var_estado_ruedas;
/*
variable de estado 3 : 
0: esta a la izquierda
1 esta recto 
2 esta a la derecha
Se inicializa en 1 porque parte recto
*/


//byte tipo dato de 0-255
byte sen_izq=A15; // sensor de la izquierda
byte sen_der=A8; // sensor de la derecha

int line;

int tiempo_enderezar=1000;
//--velocidades--//
byte vel_trasero=150; // establece la maxima velocidad
byte vel_delantero=255;

byte velocidad_maxima=255;
byte velocidad_media=170;
byte velocidad_minima=100;


boolean doblando_izq;
boolean doblando_der;
boolean avanzando;
boolean retrocediendo;

booblean doblando_recto;

/* INPUTS DE PLACA L298D*/

//Motor delantero A
byte ENA=8; 
byte IN1=9;  
byte IN2=10;

//Motor B Trasero
byte ENB=13; 
byte IN3=11; 
byte IN4=12;

//--SENTIDO DE GIRO--//
byte sentido_giro;

void setup(){

  Serial.begin(9600);
  delay(5000);
  calibrar(); // calibra poniendo en falso
  pinMode(sen_izq,INPUT);   // sensor inicia como entrada
  pinMode(sen_der,INPUT);   // snesor inicia como entrada

  /*MOTOR DELANTERO*/
  pinMode(ENA,OUTPUT);
  pinMode (IN1, OUTPUT);    // Input1 conectada al pin _ 
  pinMode (IN2, OUTPUT);    // Input2 conectada al pin _ 

  /*MOTOR DELANTERO*/
  pinMode(ENB,OUTPUT);
  pinMode (IN3, OUTPUT);    // Input3 conectada al pin _ 
  pinMode (IN4, OUTPUT);    // Input4 conectada al pin _



  byte sentido_giro=1;

  var_est=1;

  line=100;

}

void loop(){
//detecta la pista  
if((analogRead(sen_izq)>line) && (analogRead(sen_der)>line) ){ // si el sensor izquierdo y derecha detecta la linea blanca
  //rueda directa 
  avanzar();
}


if((analogRead(sen_izq)<line) && (analogRead(sen_der)<line) ){// si el sensor izquierdo y derecha detecta la linea blanca
  //ver el sentido de giro con un boton
  if (sentido==1){
    giro_der(1000,1000,1000);
    }else if(sentido==0){
      giro_izq(1000,1000,1000);
    }

  }

//Detecta el sensor de linea derecha      
if((analogRead(sen_izq)>line)  && (analogRead(sen_der)<line)){ // si el sensor derecho detecta la linea blanca 
  //Giro a la Izquierda, sera con sentidos?
  
}  


// Detecta la linea I zquierda
  if((analogRead(sen_izq)<line) && (analogRead(sen_der)>line) ){ // si el sensor izquierdo detecta la linea blanca
    //Giro a la Derecha sentidos?
    
  }


}


//trda : tiempo ruedas delanteras atras
//trta : tiempo ruedas traseras atras
//trdd : tiempo ruedas delanteras delante
void giro_der(int trda,int trta,int trdd){

  // motor delantero a un lado
  analogWrite(ENA, velocidad_maxima);
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  
  // motor trasero atras
  analogWrite(ENB, velocidad_maxima);
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
  delay(trda);

  //Dobla al sentido contrario hacia delante
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
  
  delay(trta);


  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  delayMicroseconds(tiempo_enderezar);

  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);

  delay(trdd);
}



void giro_izq(int trda,int trta,int trdd){

  // motor delantero a un lado
  analogWrite(ENA, velocidad_maxima);
  digitalWrite (IN1,  HIGH);
  digitalWrite (IN2, LOW);
  
  // motor trasero atras
  analogWrite(ENB, velocidad_maxima);
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
  delay(trda);

  //Dobla al sentido contrario hacia delante
  digitalWrite (IN1,  LOW);
  digitalWrite (IN2, HIGH);
  
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
  
  delay(trta);


  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  delayMicroseconds(tiempo_enderezar);

  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);

  delay(trdd);

}

void avanzar(){
  switch (var_estado_ruedas) {
    case 0:

      var_estado_ruedas=1;

      digitalWrite (IN1, HIGH);
      digitalWrite (IN2, LOW);
      delayMicroseconds(9000);

    break;

    case 1:

      var_estado_ruedas=1;
      //esta recto
      
    break;

    case 2:

      var_estado_ruedas=1;
      //esta en la

      digitalWrite (IN1, HIGH);
      digitalWrite (IN2, LOW);
      delayMicroseconds(9000);

    break;

    default: 
      // if nothing else matches, do the default
      // default is optional
    break;
  }
}






