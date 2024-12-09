#include <ESP32Servo.h>
#define s1_Pin 12
#define s2_Pin 13

//float corrX = 15;
//float corrY = 9.3;

int positionS1;
int positionS2;

int tiempo = 0;
#define tiempoQuieto 500
#define tiempoRecorriendo 1000

enum state {recorriendo, quieto};
enum state estado = quieto;

Servo s1;
Servo s2;

void setup() {
  s1.attach(s1_Pin);
  s2.attach(s2_Pin);
  Serial.begin(9600);
}

void loop() { 
  switch(estado){
    case recorriendo:
      if(tiempo<=0){
        tiempo = tiempoQuieto;
        estado = quieto;
      }
      funcRecorrido();
      break;
    case quieto:
      if(tiempo<=0){
        tiempo = tiempoRecorriendo;
        estado = recorriendo;
      }
      funcQuieto();
      break;
  } 
  tiempo--;
  delay(1);
}

void funcQuieto(){
  positionS1 = 0;
  positionS2 = 0;
}

void funcRecorrido(){
  s1.write(positionS1);
  s2.write(positionS2);
  /*
  if((s1.read() == positionS1)&(s2.read() == positionS2)){
    estado = quieto;
  }
  */
}
