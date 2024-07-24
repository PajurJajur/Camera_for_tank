#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;

unsigned long timer = 0;
unsigned long timer1 = 0;
float timeStep = 0.01;
int y = 0;
int i = 0;
float pitch = 0;
float roll = 0;
float yaw = 0;
float yaw1 = 0;
float pitch1 = 0;
float roll1 = 0;

unsigned long dlugosc;
unsigned long dlugosc1;
unsigned long dlugosc2;

int pwm_lew_tyl ;
int pwm_praw_tyl;
int pwm_lew_przod ;
int pwm_praw_przod;
int pin_przod_lewy =3 ;
int pin_przod_prawy =5 ;
int pin_tyl_lewy =6 ;
int pin_tyl_prawy =9 ;
void setup() {
pinMode(10, INPUT);
pinMode(4, OUTPUT);
pinMode(2, OUTPUT);
pinMode(pin_przod_lewy, OUTPUT);
pinMode(pin_przod_prawy, OUTPUT);
pinMode(pin_tyl_lewy, OUTPUT);
pinMode(pin_tyl_prawy, OUTPUT);
digitalWrite(4,HIGH);
digitalWrite(2,HIGH);
digitalWrite(pin_przod_lewy,LOW);
digitalWrite(pin_przod_prawy,LOW);
digitalWrite(pin_tyl_lewy,LOW);
digitalWrite(pin_tyl_prawy,LOW);

pinMode(11, INPUT);
pinMode(12, INPUT);
//Serial.begin(9600);
Serial.begin(115200);

while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_4G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
 //mpu.calibrateGyro(); 

 //mpu.setThreshold(3); 

}

void loop() {
//Pomiar impulsów --> konwersja na pwm z arduino

dlugosc = pulseIn(10,HIGH);

dlugosc1 = pulseIn(11,HIGH);

dlugosc2 = pulseIn(12,HIGH);
pwm_lew_przod = map(dlugosc, 1000, 2000,-255,255 );
pwm_praw_przod = map(dlugosc1, 1000, 2000,-255,255 );
pwm_lew_tyl = map(dlugosc, 2000, 1000,-255,255 );
pwm_praw_tyl = map(dlugosc1, 2000, 1000,-255,255 );

// Ograniczenia Pwm 

if(pwm_lew_przod <30)
{
  pwm_lew_przod = 0;
}
if(pwm_lew_tyl <30)
{
  pwm_lew_tyl = 0;
}
if(pwm_lew_przod >255)
{
  pwm_lew_przod = 255;
}
if(pwm_lew_tyl >255)
{
  pwm_lew_tyl = 255;
}

if(pwm_praw_przod <30)
{
  pwm_praw_przod = 0;
}
if(pwm_praw_tyl <30)
{
  pwm_praw_tyl = 0;
}
if(pwm_praw_przod >255)
{
  pwm_praw_przod = 255;
}
if(pwm_praw_tyl >255)
{
  pwm_praw_tyl = 255;
}

// 

if(dlugosc2>1600)
{
  obracanie();
}

//Regulacja PWM

analogWrite(pin_przod_lewy, pwm_lew_przod);
analogWrite(pin_przod_prawy,pwm_praw_przod);
analogWrite(pin_tyl_lewy,pwm_lew_tyl);
analogWrite(pin_tyl_prawy,pwm_praw_tyl);

//Test na terminalu

//Serial.println(dlugosc);
//Serial.println("-----");
//Serial.println(dlugosc1);
//Serial.println("-----");
//Serial.println(dlugosc2);
//Serial.println("-----");
//Serial.println("lewo_tyl");
//Serial.println(pwm_lew_tyl);
//Serial.println("prawo_tyl");
//Serial.println(pwm_praw_tyl);
//Serial.println("lewo_przod");
//Serial.println(pwm_lew_przod);
//Serial.println("prawo_przod");
//Serial.println(pwm_praw_przod);
//Serial.println("\n\n\n\n\n");
//delay(500);

}

// funkcja odpowiedzialna za skręt w prawo o 90 stopni

void skret_w_prawo()
{
mpu.calibrateGyro();
mpu.setThreshold(3);
yaw = 0;
timer = millis();

  // odczytaj znormalizowane wartości kątów
  Vector norm = mpu.readNormalizeGyro();

  // kąty obrotów w osiach x, y, z
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  // opóźnienie pomiarowe
  delay((timeStep*1000) - (millis() - timer));

  yaw1=0;
  i=0;
  while(abs(yaw1)<abs(yaw)+85)
  {
    timer1 = millis();

    // odczytaj znormalizowane wartości kątów
    Vector norm1 = mpu.readNormalizeGyro();

    // kąty obrotów w osiach x, y, z
    pitch1 = pitch1 + norm1.YAxis * timeStep;
    roll1 = roll1 + norm1.XAxis * timeStep;
    yaw1= yaw1 + norm1.ZAxis * timeStep;

    //Serial.print(" Pitch = ");
    //Serial.print(pitch1);
    //Serial.print(" Roll = ");
    //Serial.print(roll1);  
    //Serial.print(" Yaw = ");
    //Serial.println(yaw1);
     // opóźnienie pomiarowe
    delay((timeStep*1000) - (millis() - timer1));
    //i++;
    //if(i>=150) i =150;
    analogWrite(pin_tyl_prawy,100);
    analogWrite(pin_przod_lewy,100);
    
    

  }
  analogWrite(pin_tyl_prawy,0);
  analogWrite(pin_przod_lewy,0);
  analogWrite(pin_przod_prawy,100);
  analogWrite(pin_tyl_lewy,100);
  delay(200);
  analogWrite(pin_przod_prawy,0);
  analogWrite(pin_tyl_lewy,0);

}

//autonomiczny test kwadratu

void obracanie()
{
  skret_w_prawo();
  delay(1000);
  for(y=0;y<=255;y++){
  analogWrite(pin_tyl_prawy,y);
  analogWrite(pin_tyl_lewy,y);
  delay(10);
  }
  
  delay(4000);
  analogWrite(pin_tyl_prawy,0);
  analogWrite(pin_tyl_lewy,0);
  delay(1000);
  /*
  skret_w_prawo();
  delay(500);
  for(y=0;y<=255;y++){
  analogWrite(pin_tyl_prawy,y);
  analogWrite(pin_przod_lewy,y);
  delay(10);
  }
  
  delay(1000);
  analogWrite(pin_tyl_prawy,0);
  analogWrite(pin_przod_lewy,0);
  delay(500);
    skret_w_prawo();
  delay(500);
  for(y=0;y<=255;y++){
  analogWrite(pin_tyl_prawy,y);
  analogWrite(pin_przod_lewy,y);
  delay(10);
  }
  
  delay(1000);
  analogWrite(pin_tyl_prawy,0);
  analogWrite(pin_przod_lewy,0);
  delay(500);
    skret_w_prawo();
  delay(500);
  for(y=0;y<=255;y++){
  analogWrite(pin_tyl_prawy,y);
  analogWrite(pin_przod_lewy,y);
  delay(10);
  }
  
  delay(1000);
  analogWrite(pin_tyl_prawy,0);
  analogWrite(pin_przod_lewy,0);
  delay(500);
*/

}
