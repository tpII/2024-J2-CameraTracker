#include <Servo.h>
#define s1_Pin 5
#define s2_Pin 6
#define pA 0
#define pB 90
#define pC 180
#define pD 89

int Position = pA;

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
      funcRecorrido();
      break;
    case quieto:
      funcQuieto();
      break;
  } 
}

void funcQuieto(){
  switch(Position){
    case pA:
      Position = pB;
      break;
    case pB:
      Position = pC;
      break;
    case pC:
      Position = pD;
      break;
    case pD:
      Position = pA;
      break;
  }
  estado = recorriendo;
  delay(500);
}

void funcRecorrido(){
  s1.write(Position);
  s2.write(Position);
  if((s1.read() == Position)&(s2.read() == Position)){
    estado = quieto;
  }
}
