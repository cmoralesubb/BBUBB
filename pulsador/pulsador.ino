
#define PULSADOR 22
#define LED 24
 
int estadoPulsador = 0;
 
void setup() {
  Serial.begin(9600);
  pinMode(PULSADOR, INPUT);


}
 
void loop() {
  estadoPulsador = digitalRead(PULSADOR);
  Serial.println(estadoPulsador);
}
