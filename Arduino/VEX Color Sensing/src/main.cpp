#include <Arduino.h>
#include "Adafruit_TCS34725.h"


int redPort = 2;
int bluePort = 9;

int colorDiff = 0;

float ambientColor[3];
float r, g, b;

#define COLOR_MARGIN 30

void initializeColorSensor(int redPort, int bluePort);
int calibrateColorSensor();
void readColor();
bool colorSeesRed();
bool colorSeesBlue();

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  
  initializeColorSensor(redPort, bluePort);
  
}


void loop() {
  // put your main code here, to run repeatedly:

  readColor();

  if (colorSeesRed()) {
    digitalWrite(redPort, HIGH);
    digitalWrite(bluePort, LOW);
  }
  else if (colorSeesBlue()) {
    digitalWrite(bluePort, HIGH);
    digitalWrite(redPort, LOW);
  }
  else {
    digitalWrite(redPort, LOW);
    digitalWrite(bluePort, LOW);
  }

  delay(10);

}


/*
* Initializes the FLORA TCS34725 color sensor. A4 and A5 MUST be used for the color sensor I/O
*/
void initializeColorSensor(int redPort, int bluePort) {

  pinMode(redPort, OUTPUT);
  pinMode(bluePort, OUTPUT);
  
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");

    pinMode(13, OUTPUT);
    while (1) {
      digitalWrite(13, HIGH);
      delay(60);
      digitalWrite(13, LOW);
      delay(60);
    }
  }

  Serial.print("R:\t"); Serial.print(int(r)); 
  Serial.print("\tG:\t"); Serial.print(int(g)); 
  Serial.print("\tB:\t"); Serial.print(int(b));

  colorDiff = calibrateColorSensor();
}

int calibrateColorSensor() {
  tcs.getRGB(&ambientColor[0], &ambientColor[1], &ambientColor[2]);

  return ambientColor[0] - ambientColor[2];
}

void readColor() {

  tcs.setInterrupt(false);  // turn on LED

  delay(50);  // takes 50ms to read

  tcs.getRGB(&r, &g, &b);

  //tcs.setInterrupt(true);  // turn off LED
}

bool colorSeesRed() {
  return ((r - colorDiff) > (b + COLOR_MARGIN));
}

bool colorSeesBlue() {
  return (b > ((r - colorDiff) + COLOR_MARGIN));
}
