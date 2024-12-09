#include <ESP32Servo.h>
#define s1_Pin 12 //Pin del servo X
#define s2_Pin 13 //Pin del servo Y

float MAX_X = 80; //Posicion maxima en X
float MAX_Y = 70; //Posicion maxima en Y

float DIST_X; //Distancia X del objeto al centro de la imagen
float DIST_Y; //Distancia Y del objeto al centro de la imagen

float FOV_X = 90; //Angulo FOV en X (Grados)
float FOV_Y = 80; //Angulo FOV en Y (Grados)

float ANG_X; //Angulo de corrimiento del objeto al centro de la imagen
float ANG_Y; //Angulo de corrimiento del objeto al centro de la imagen

int positionS1; //Posicion del Servo 1
int positionS2; //Posicion del Servo 2

Servo s1;
Servo s2;

void setup() {
  s1.attach(s1_Pin);
  s2.attach(s2_Pin);
  Serial.begin(9600);
}

void loop() {
  float X = 5; //Posicion del objeto con respecto al lado izquierdo de la imagen
  float Y = 5; //Posicion del objeto con respecto al lado superior de la imagen
  
  DIST_X = X - (MAX_X / 2);
  DIST_Y = -Y + (MAX_X / 2);

  ANG_X = DIST_X * FOV_X / MAX_X;
  ANG_Y = DIST_Y * FOV_Y / MAX_Y;

  positionS1 = ANG_X + 90; //el +90 es por la posicion media del servo
  positionS2 = ANG_Y + 90; //el +90 es por la posicion media del servo

  s1.write(positionS1);
  s2.write(positionS2);

  delay(1000);
}