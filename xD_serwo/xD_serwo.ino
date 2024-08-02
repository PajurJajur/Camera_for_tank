#include <Servo.h>
#include <string.h>
#include <Arduino.h>

Servo Servo1;
Servo Servo2;
int x = 0;
int y = 135;
int z = 0;
int t = 90;



void setup() {
  // put your setup code here, to run once:

Serial.begin(9600);
Servo1.attach(6);
Servo2.attach(9);
Servo1.write(135);
Servo2.write(90);
delay(1000);


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

    t= map(value1,0,1920,135,45);
    y= map(value2,0,1080,180,90);
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
  if(y>155){
    y=155;
  }

  if(y<90){
    y=90;
  }

  if(t>135){
    t=135;
  }

  if(t<45){
    t=45;
  }

 // Servo1.write(y);
 // Servo2.write(t);
 delay(15);



}
