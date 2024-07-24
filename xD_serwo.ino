#include <Servo.h>
#include <string.h>
#include <Arduino.h>

Servo Servo1;
Servo Servo2;
int x = 0;
int y = 0;
int z = 0;
int t = 0;



void setup() {
  // put your setup code here, to run once:
pinMode(4, OUTPUT);
Serial.begin(9600);
Servo1.attach(6);
Servo2.attach(9);
digitalWrite(4, HIGH);
Servo1.write(125);
Servo2.write(125);


}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(Serial.available())
  {
     String data = Serial.readStringUntil('\n');
        int commaIndex = data.indexOf(',');

        if (commaIndex > 0) {
            int value1 = data.substring(0, commaIndex).toInt();
            int value2 = data.substring(commaIndex + 1).toInt();

    y= map(value1,0,1920,0,180);
    t= map(value2,0,1080,0,180);
    /*
    if(odczyt == "A")
      {
        x++; 
      }
    
    if(odczyt == "B")
      {
        x--;
      }
  */
 
  }}
  if(y>255){
    y=255;
  }

  if(y<0){
    y=0;
  }

  if(t>255){
    t=255;
  }

  if(t<0){
    t=0;
  }

  Servo1.write(y);
  Servo2.write(t);



}