#include <iostream>
#include <unistd.h>
#include <pigpio.h>
#include <wiringPi.h>
#include "rotary_encoder.hpp"
#include <wiringPiI2C.h> 
#include <stdlib.h>
#include "czujniki.cpp"
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace cv;

int fd ;
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
float integralLimit = 100.0; 

float prev_errorL = 0, prev_errorR = 0;
float integralL = 0, integralR = 0;


int hmin = 19, smin = 2, vmin = 176;
int hmax = 94, smax = 155, vmax = 255;

Mat image, image1, imagewykryty;


void setup_PWM(int pin){
        //10kHz from 0 to 255
        gpioSetMode(pin, PI_OUTPUT);  
        gpioSetPWMfrequency(pin,10000);
        gpioSetPWMrange(pin, 255);
}

void set_DIR_PIN(int pin, int state){
        gpioSetMode(pin, PI_OUTPUT);
        gpioWrite(pin, state);
}

int camera_init(){
 VideoCapture Cam(0);
    usleep(2000000); // 20 ms
    if (!Cam.isOpened()) {
        std::cerr << "Nie można otworzyć kamery\n";
        return 1;
    }
    return 0;
}


void detect_cone(){

 Cam.read(image);
        if (image.empty()) {
            cerr << "Nie można odczytać obrazu z kamery\n";
            break;
        }

        cvtColor(image, image1, COLOR_BGR2HSV);
        Scalar dolna(hmin, smin, vmin);
        Scalar gorna(hmax, smax, vmax);

        inRange(image1, dolna, gorna, imagewykryty);

        // Filtracja szumów
        erode(imagewykryty, imagewykryty, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        dilate(imagewykryty, imagewykryty, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

        dilate(imagewykryty, imagewykryty, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        erode(imagewykryty, imagewykryty, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

        Moments oMoments = moments(imagewykryty);
        double dM01 = oMoments.m01;
        double dM10 = oMoments.m10;
        double dArea = oMoments.m00;

        if (dArea > 10000) {
            int posX = dM10 / dArea;
            int posY = dM01 / dArea;

            int centerX = image.cols / 2;
            int centerY = image.rows / 2;

            float distance = calculateDistance(posX, posY, centerX, centerY);
            
            
            int errorX = centerX - posX;
            int errorY = centerY - posY;

	integralX += errorX;
            integralY += errorY;

            int dErrorX = errorX - prevErrorX;
            int dErrorY = errorY - prevErrorY;



     ellipse(image, Point(posX, posY), Size(100, 100), 0, 0, 360, Scalar(255, 0, 0), 6);
            line(image, Point(posX - 100, posY), Point(posX + 100, posY), Scalar(255, 0, 0), 3);
            line(image, Point(posX, posY - 100), Point(posX, posY + 100), Scalar(255, 0, 0), 3);
        }

        //imshow("camera", image);
        //imshow("camera1", imagewykryty);
        if (waitKey(1) == 27) break;
}



void turn_left_right(float angle_to_achieve, float current_angle, float error, int pwm, int pin_R, int pin_L, int dir1, int dir2) {

    if (angle_to_achieve - current_angle > 0) {  // Obrót w prawo
        set_DIR_PIN(dir1, 0);  
        set_DIR_PIN(dir2, 1);
    } else {  // Obrót w lewo
        set_DIR_PIN(dir1, 1);  
        set_DIR_PIN(dir2, 0);
    }
    gpioPWM(pin_L, pwm);
    gpioPWM(pin_R, pwm);

    while (fabs(angle_to_achieve - current_angle) > error) {
        i2c_read_gyro(fd,&angleX,&angleY,&angleZ,dt);
        current_angle = angleZ;
    }

    gpioPWM(pin_L, 0);
    gpioPWM(pin_R, 0);
}

void drive_straight(int rpm_to_achive, double current_rpmL,double current_rpmR, int pin_R, int pin_L){
    float integralR;
    float integralL;


    float errorR = rpm_to_achive - current_rpmR;
    float errorL = rpm_to_achive - current_rpmL;

    integralR += errorR;
    integralL += errorL;

    if (integralL > integralLimit) integralL = integralLimit;
    else if (integralL < -integralLimit) integralL = -integralLimit;

    if (integralR > integralLimit) integralR = integralLimit;
    else if (integralR < -integralLimit) integralR = -integralLimit;

    float derivativeR = errorR - prev_errorR;
    float derivativeL = errorL - prev_errorL;

    int dutyR = errorR *Kp + integralR *Ki + derivativeR * Kd;
    int dutyL = errorL *Kp + integralL *Ki + derivativeL * Kd;


    //odpowiednik map z arduino
    dutyL = std::max(0, std::min(255, dutyL));
    dutyR = std::max(0, std::min(255, dutyR));

    float prev_errorR = errorR;
    float prev_errorL = errorL;

    gpioPWM(pin_L, dutyL);  // Silnik lewy
    gpioPWM(pin_R, dutyR);   // Silnik prawy
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

    fd = i2C_init();

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
