#include <Servo.h>
#include <string.h>
#include <Arduino.h>

Servo Servo1;
Servo Servo2;
int x = 0;
int y = 162;
int z = 0;
int t = 90;



void setup() {
  // put your setup code here, to run once:

Serial.begin(9600);
Servo1.attach(6);
Servo2.attach(9);
Servo1.write(162);
Servo2.write(90);


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

    y= map(value1,0,1920,45,135);
    t= map(value2,0,1080,180,115);
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
  if(y>135){
    y=135;
  }

  if(y<45){
    y=45;
  }

  if(t>180){
    t=180;
  }

  if(t<115){
    t=115;
  }

  //Servo1.write(y);
 //Servo2.write(t);



}