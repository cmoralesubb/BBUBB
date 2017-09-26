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
  byte sen_izq = 5; // sensor de la izquierda
  byte sen_der = 6; // sensor de la derecha
  byte sen_tras = 7;
  
  int pista;
  
  int tiempo_enderezar = 1000;
  //--velocidades--//
  byte vel_trasero = 150; // establece la maxima velocidad
  byte vel_delantero = 255;
  
  byte velocidad_maxima = 255;
  byte velocidad_media = 255;
  byte velocidad_minima = 100;
  
  
  boolean doblando_izq;
  boolean doblando_der;
  boolean avanzando;
  boolean retrocediendo;
  
  boolean doblando_recto;
  
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
  
  
  int sensor_izquierdo;
  int sensor_derecho;
  int sensor_trasero;
  
  
  void setup() {
    Serial1.begin(38400);
    Serial.begin(9600);
  
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
  
    byte sentido_giro = 1;
  
    pista = 1;
  
  }
  
  void loop() {
    //detecta la pista
    char c = Serial1.read();
  
    if (c == 'g') {
      digitalWrite (IN1, LOW);
      digitalWrite (IN2, LOW);
  
      digitalWrite (IN3, LOW);
      digitalWrite (IN4, LOW);
    }
    if (c == 'a') {
  
      Serial.println("Entro a c");
      sensor_izquierdo = digitalRead(sen_izq);
      sensor_derecho = digitalRead(sen_der);
      sensor_trasero = digitalRead(sen_tras);


      if ((sensor_izquierdo == pista) && (sensor_derecho == pista)  && (sensor_trasero == pista)) { // si el sensor izquierdo y derecha detecta la pistaa blanca
        //rueda directa
        avanzar();
      }
  
  
      if ((sensor_izquierdo < pista) && (sensor_derecho < pista)) { // si el sensor izquierdo y derecha detecta la pistaa blanca
        //ver el sentido de giro con un boton
        if (sentido_giro == 1) {
          giro_der();
        } else if (sentido_giro == 0) {
          giro_izq();
        }
      }
  
      //Detecta el sensor de pistaa derecha
      if ((sensor_izquierdo == pista)  && (sensor_derecho == pista)) { // si el sensor derecho detecta la pistaa blanca
        //Giro a la Izquierda, sera con sentidos?
  
      }
  
  
      // Detecta la pistaa I zquierda
      if ((sensor_izquierdo < pista) && (sensor_derecho == pista) && (sensor_trasero == pista)) { // si el sensor izquierdo detecta la pistaa blanca
        //Giro a la Derecha sentidos?
        emparejar_derecha();
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
    Serial.println("Entro al avanzar");
    digitalWrite (IN3, LOW);
    digitalWrite (IN4, HIGH);
    analogWrite(ENB, velocidad_maxima);
  }
  
  //trda : tiempo ruedas delanteras atras
  //trta : tiempo ruedas traseras atras
  //trdd : tiempo ruedas delanteras delante
  void giro_der() {
  
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
  
  void emparejar_derecha() {
    Serial.println("Emparejar_derecha");
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
  
  
  void emparejar_izquierda() {
    //lo mismo pero al otro lado
  }
  
  void giro_izq() {
  
    //Motor trasero sigue su rumbo
  
    //Motor trasero sigue su rumbo
    //Motor delantero a un lado
  
    digitalWrite (IN1, LOW);
    digitalWrite (IN2, HIGH);
    analogWrite(ENA, velocidad_maxima);
  
    // motor trasero atras
    analogWrite(ENB, velocidad_media);
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
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    delay(3000);
  
  }
  
  
  
  
  
  /*
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
    }
  */
  
  
  

