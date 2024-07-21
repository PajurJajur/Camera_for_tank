#include <Servo.h>
#include <string.h>
#include <Arduino.h>

Servo Servo1;
Servo Servo2;
int x = 0;
int y = 0;
int z = 0;
int t = 0;


String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return data.substring(strIndex[0], strIndex[1]);
}



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
    String odczyt = Serial.readStringUntil('\n');
    int pos1 = getValue(odczyt, ',', 0).toInt();
    int pos2 = getValue(odczyt, ',', 1).toInt();
    y= map(pos1,0,640,0,255);
    t= map(pos2,0,480,0,255);
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
 
  }
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
