#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <ESP32Servo.h>

#define S1_PIN 12 // Pin del servo X
#define S2_PIN 13 // Pin del servo Y

Servo s1;
Servo s2;

void setup() {
  s1.attach(S1_PIN);
  s2.attach(S2_PIN);
}

void loop() {
  s1.write(30);
  s2.write(0);
  vTaskDelay(2000);
  s1.write(150);
  s2.write(180);
  vTaskDelay(2000);
}
