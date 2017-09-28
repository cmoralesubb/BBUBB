  /*
    EVALUAR SI LEER ANTES LAS VARIABLES DE DATOS
    VER VUELTAS CORTAS DE GIRAR
    */

  /*
    EXPO ROBOTICA UBB 2017
    MATIAS VIDAL GUTIERREZ , CLAUDIO MORALES GUTIERREZ
    IECI
    */

  /*
    variable de estado 3 :
    0: esta a la izquierda
    1 esta recto
    2 esta a la derecha
    Se inicializa en 1 porque parte recto
    */


  //byte tipo dato de 0-255
  byte sen_izq = 5; // sensor de la izquierda
  byte sen_der = 6; // sensor de la derecha
  
  byte velocidad_maxima = 255;
  byte velocidad_media = 200;
  byte velocidad_minima = 100;
  
  byte recto=0;
  byte subida=-1;
  byte bajada=1;
  
  
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
  
  int pista;

  int tiempo_enderezar = 1000;

  
  void setup() {

    Serial.begin(9600);
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


  }

  void loop() {
      sensor_izquierdo=digitalRead(sen_izq);
      sensor_derecho=digitalRead(sen_der);
      sensor_trasero=digitalRead(sen_tras);      
      Serial.println(sensor_izquierdo);


        if ((sensor_izquierdo != pista) && (sensor_derecho != pista)) { // si el sensor izquierdo y derecha detecta la pistaa blanca
        //ver el sentido de giro con un boton
        if(contacto==1){
          if (sentido_giro == 1) {
              Serial.println("giro derecha");
              giro_der();
            } else if (sentido_giro == 0) {
              Serial.println("giro izquierda");
              giro_izq();
            }          
          }
          contacto=0;
        }
  
      if ((sensor_izquierdo == pista) && (sensor_derecho == pista) ) { // si el sensor izquierdo y derecha detecta la pistaa blanca
        //rueda directa
        Serial.println("avanzando");
        avanzar();
      }

      //Detecta el sensor de pistaa derecha
      if ((sensor_izquierdo == pista)  && (sensor_derecho != pista) ) { // si el sensor derecho detecta la pistaa blanca
        //Giro a la Izquierda, sera con sentidos
        contacto=1;
        Serial.println("emparejar izquierda");
        emparejar_izquierda();
      }

      // Detecta la pistaa I zquierda
      if ((sensor_izquierdo != pista) && (sensor_derecho == pista) ) { // si el sensor izquierdo detecta la pistaa blanca
        //Giro a la Derecha sentidos?
        contacto=1;
        Serial.println("emparejar derecha");
        emparejar_derecha();
      }

    delay(50);      
    
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
      analogWrite(ENB, velocidad_maxima);
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

    delay(1000);
   
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

    delay(1000);


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


