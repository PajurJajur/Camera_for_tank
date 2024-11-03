#include <iostream>
#include <unistd.h>
#include <pigpio.h>
#include <wiringPi.h>
#include "rotary_encoder.hpp"
#include <wiringPiI2C.h> 
#include <stdlib.h>
#include "czujniki.cpp"


int posR = 0;  
int posL = 0;  
double rpmR = 0;  
double rpmL = 0;
double distance = 0; 
double distance1 = 0; 
double distance2 = 0; 
double distance3 = 0;
float angleX = 0;
float angleY = 0;
float angleZ = 0;
float dt = 0.2;
const double IMPULSY_NA_OBROT = 1633.25;  
float Kp = 1.0;   
float Ki = 0.1;  
float Kd = 0.05;  

float prev_errorL = 0, prev_errorR = 0;
float integralL = 0, integralR = 0;

void drive_straight(int rpm_to_achive, double current_rpmL,double current_rpmR){

    float errorR = rpm_to_achive - current_rpmR;
    float errorL = rpm_to_achive - current_rpmL;

    float integralR += errorR + prev_errorR;
    float integralL += errorL + prev_errorL;

    float derivativeR = errorR - prev_errorR;
    float derivativeL = errorL - prev_errorL;

    int dutyR = errorR *Kp + integralR *Ki + derivativeR * Kd;
    int dutyL = errorL *Kp + integralL *Ki + derivativeL * Kd;


    //odpowiednik map z arduino
    dutyL = std::max(0, std::min(255, dutyL));
    dutyR = std::max(0, std::min(255, dutyR));

    float prev_errorR = errorR;
    float prev_errorL = errorL;

    gpioPWM(10, dutyL);  // Silnik lewy
    gpioPWM(9, dutyR);   // Silnik prawy
}



void callback(int wayR) {
    posR += wayR;
}

void callback2(int wayL) {
    posL += wayL;
}



double calculateRPM(int impulsy, double czas_s) {
    double obroty_na_sekunde = impulsy / IMPULSY_NA_OBROT / czas_s;
    return obroty_na_sekunde * 60.0;  // Konwersja na obroty na minutę (RPM)
}




int main(int argc, char *argv[]) {
    if (gpioInitialise() < 0) return 1;

    re_decoder dec(16, 17, callback);
    re_decoder dec2(23, 24, callback2);

    int fd = i2C_init();

    uint32_t start_time = gpioTick();

    while (true) {

        uint32_t current_time = gpioTick();
        double elapsed_s = (current_time - start_time) / 1e6;  // Przeliczenie z mikrosekund na sekundy

        if (elapsed_s >= 0.1) {  // czas co jaki pomiar
            rpmR = calculateRPM(posR, elapsed_s);
            rpmL = calculateRPM(posL, elapsed_s);
            distance = distance_sensor_measure(4);
            usleep(500);
            distance1 = distance_sensor_measure(27);
            //usleep(500);
            //distance2 = distance_sensor_measure(22);
            //usleep(500);
            //distance3 = distance_sensor_measure(10);

            i2c_read_gyro(fd,&angleX,&angleY,&angleZ,dt);
            std::cout << "AngleX: " << angleX << "AngleY: " << angleY << "AngleZ: " << angleZ << std::endl;
            std::cout << "Odleglosc1: " << distance << std::endl;
            std::cout << "Odleglosc2: " << distance1 << std::endl;
            std::cout << "Impulsy=" << posR << "  Czas=" << elapsed_s << " sek  RPM=" << rpmR << std::endl;
            std::cout << "Impulsy ten drugi=" << posL << "  Czas=" << elapsed_s << " sek  RPM=" << rpmL<< std::endl;


            gpioSetPWMfrequency(10,10000);
            gpioSetPWMrange(10, 255);
            gpioPWM(10, duty);
            gpioSetPWMfrequency(9,10000);
            gpioSetPWMrange(9, 255);
            gpioPWM(9, duty);

            posR = 0;
            posL = 0;
            start_time = gpioTick();
        }
        usleep(10000);  // 10 ms
    }

    dec.re_cancel();
    dec2.re_cancel();
    gpioTerminate();
}
