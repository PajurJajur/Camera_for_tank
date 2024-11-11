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
int dir1 = 13;
int dir2 = 19;
int pin_R = 20;
int pin_L = 21;
double distance = 0; 
double distance1 = 0; 
double distance2 = 0; 
double distance3 = 0;

float alpha = 0.8;
float angleX = 0;
float angleY = 0;
float angleZ = 0;
float dt = 0.055;
const double IMPULSY_NA_OBROT = 1633.25;  
//PID od jechania na wprost
float Kp_R = 4.0;
float Ki_R =0.2;
float Kd_R = 0.0;

float Kp_L = 4.0;
float Ki_L = 0.2;
float Kd_L = 0.0;

float integralLimit = 800.0; // mniejsze ograniczenie dla całki
//PID od lokalizacji pachołka
float Kp1 = 0.3;   
float Ki1 = 0.05;  
float Kd1 = 0.01;



double dArea=0;
int centerX=0;
int posX=0;
float prevErrorR = 0; 
float integralR = 0;
float prevErrorL = 0; 
float integralL = 0;
float prevErrorX = 0; 
float integralX = 0;
float prevErrorO = 0; 
float integralO = 0;
float prevErrorC = 0; 
float integralC = 0;

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


void detect_cone(Mat& image, int& posX, double& dArea, int& centerX) {
    cvtColor(image, image1, COLOR_BGR2HSV);
    Scalar lower(hmin, smin, vmin);
    Scalar upper(hmax, smax, vmax);

    inRange(image1, lower, upper, imagewykryty);

    // Filtracja szumów
    erode(imagewykryty, imagewykryty, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(imagewykryty, imagewykryty, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    dilate(imagewykryty, imagewykryty, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(imagewykryty, imagewykryty, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    Moments oMoments = moments(imagewykryty);
    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    dArea = oMoments.m00;

    if (dArea > 10000) {
        posX = dM10 / dArea;
        int posY = dM01/ dArea;
        centerX = image.cols / 2;
        ellipse(image, Point(posX, posY), Size(100, 100), 0, 0, 360, Scalar(255, 0, 0), 6);
        line(image, Point(posX - 100, posY), Point(posX + 100, posY), Scalar(255, 0, 0), 3);
        line(image, Point(posX, posY - 100), Point(posX, posY + 100), Scalar(255, 0, 0), 3);
    }
}


float compute_pid(float error, float& prev_error, float& integral, float kp, float ki, float kd) {
    integral += error;
    if (integral > integralLimit) integral = integralLimit;
    if (integral < -integralLimit) integral = -integralLimit;
    float derivative = error - prev_error;
    float output = kp * error + ki * integral + kd * derivative;
    prev_error = error;
    return output;
}



void adjust_movement(float output, float error) {
    int pwm_left =  fabs(output);  // PWM lewego silnika
    int pwm_right = fabs(output); // PWM prawego silnika

    pwm_left = std::clamp(pwm_left, 0, 255);
    pwm_right = std::clamp(pwm_right, 0, 255);

    if (error > 0) { // Jeśli obiekt jest po lewej stronie
        set_DIR_PIN(dir1,0); // Ustaw kierunek dla lewego silnika
        set_DIR_PIN(dir2, 0); // Ustaw kierunek dla prawego silnika
        std::cout << "Skrecam w lewo " << std::endl;
    } else { // Jeśli obiekt jest po prawej stronie
        set_DIR_PIN(dir1, 1); // Ustaw kierunek dla lewego silnika
        set_DIR_PIN(dir2, 1); // Ustaw kierunek dla prawego silnika
        std::cout << "Skrecam w prawo " << std::endl;
    }
    std::cout << "pwm_lewo "<< pwm_left << std::endl;
        std::cout << "pwm_prawo "<< pwm_right << std::endl;
    gpioPWM(pin_L, pwm_left);
    gpioPWM(pin_R, pwm_right);
}


int camera_turning(int& posX,int& centerX,double& dArea){
float error = centerX - posX;
if(fabs(error)>10){
error = centerX - posX;
output = compute_pid(error,prev_errorC,integralC,Kp1, float Ki1, float Kd1);
adjust_movement(output, error);
}else return 1;
}

void turning_better(float angle_to_achive, int& fd) {
    i2c_read_gyro(fd, &angleX, &angleY, &angleZ, dt, alpha);

    // Ustal kąt docelowy względem zakresu 0–360
    angle_to_achive = fmod(angle_to_achive, 360.0f);
    if (angle_to_achive < 0) {
        angle_to_achive += 360.0f;
    }

    float error = angle_to_achive - angleZ;

    // Wyznacz najkrótszą ścieżkę obracania
    if (error > 180) {
        error -= 360;
    } else if (error < -180) {
        error += 360;
    }

    // Pętla sterująca
    while (fabs(error) > 0.5) {
        i2c_read_gyro(fd, &angleX, &angleY, &angleZ, dt, alpha);

        error = angle_to_achive - angleZ;

        // Wyznacz najkrótszą ścieżkę obracania
        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        std::cout << "Error: " << error << std::endl;

        float output = compute_pid(error, prevErrorO, integralO, Kp1, Ki1, Kd1);
        std::cout << "Output: " << output << std::endl;

        adjust_movement(output, error);
        
        gpioDelay(50000);
    }

    set_DIR_PIN(dir1, !gpioRead(dir1));
    set_DIR_PIN(dir2, !gpioRead(dir2));
    gpioPWM(pin_L, 50);  // Niski poziom PWM dla aktywnego hamowania
    gpioPWM(pin_R, 50);
    gpioDelay(100000);   // Krótka przerwa na hamowanie
    gpioPWM(pin_L, 0);
    gpioPWM(pin_R, 0);
}


void drive_straight(int rpm_to_achive, double current_rpmL,double current_rpmR){

    float errorR = rpm_to_achive - fabs(current_rpmR);
    float errorL = rpm_to_achive - fabs(current_rpmL);
    std::cout << "Error R: " << errorR << std::endl;
    std::cout << "Error L: " << errorL << std::endl;
    int dutyR = compute_pid(errorR, prevErrorR, integralR, Kp_R, Ki_R, Kd_R);
    int dutyL = compute_pid(errorL, prevErrorL, integralL, Kp_L, Ki_L, Kd_L);

    //odpowiednik map z arduino
    dutyL = std::max(0, std::min(255, dutyL));
    dutyR = std::max(0, std::min(255, dutyR));
    // to jest na wprost kierunek
    set_DIR_PIN(dir1, 0);  //R
    set_DIR_PIN(dir2, 1);  //L
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


void sensors_measure(double time, double& distance, double& distance1, double& distance2, double& distance3,double& rpmR, double& rpmL, uint32_t& ex_start_time){

        uint32_t current_time = gpioTick();
        double elapsed_s = (current_time - ex_start_time) / 1e6;  // Przeliczenie z mikrosekund na sekundy

        if (elapsed_s >= time) {  // czas co jaki pomiar
            rpmR = calculateRPM(posR, elapsed_s);
            rpmL = calculateRPM(posL, elapsed_s);
            distance = distance_sensor_measure(4);
            gpioDelay(500);
            distance1 = distance_sensor_measure(17);
            //usleep(500);
            //distance2 = distance_sensor_measure(22);
            //usleep(500);
            //distance3 = distance_sensor_measure(10);

            std::cout << "Odleglosc1: " << distance << std::endl;
            std::cout << "Odleglosc2: " << distance1 << std::endl;
            std::cout << "Impulsy=" << posR << "  Czas=" << elapsed_s << " sek  RPM=" << rpmR << std::endl;
            std::cout << "Impulsy ten drugi=" << posL << "  Czas=" << elapsed_s << " sek  RPM=" << rpmL<< std::endl;

            posR = 0;
            posL = 0;
            ex_start_time = gpioTick();
        }
}

int main(int argc, char *argv[]) {
    if (gpioInitialise() < 0) return 1;

    re_decoder dec(5, 6, callback);
    re_decoder dec2(9, 11, callback2);

    fd = i2C_init();
    setup_PWM(int pin_R);
    setup_PWM(int pin_L);
    uint32_t start_time = gpioTick();

    VideoCapture Cam(0);
    gpioDelay(200000); // 200 ms
    if (!Cam.isOpened()) {
        std::cerr << "Nie można otworzyć kamery\n";
        return 1;
    }
    gpioDelay(50000);
    while (done !=1) {

        Cam.read(image);
        if (image.empty()) {
            std::cerr << "Nie można odczytać obrazu z kamery\n";
            break;
        }

        detect_cone(image, posX, dArea, centerX);
        int done = camera_turning(posX,centerX,dArea);
    }

    while(distace>=30){
            drive_straight(50,rpmL,rpmR);
            sensors_measure(0.1,distance, distance1, distance2, distance3, rpmR, rpmL, start_time);
            gpioDelay(50000);
    }
    gpioPWM(pin_L, 0);  
    gpioPWM(pin_R, 0);   

    turning_better(45.0, fd);
    gpioDelay(500000);
    drive_straight(50,rpmL,rpmR);
    gpioDelay(2000000);
    gpioPWM(pin_L, 0);  
    gpioPWM(pin_R, 0);
    turning_better(315.0, fd);
    drive_straight(50,rpmL,rpmR);
    gpioDelay(2000000);
    gpioPWM(pin_L, 0);  
    gpioPWM(pin_R, 0);
    turning_better(0, fd);


    dec.re_cancel();
    dec2.re_cancel();
    gpioTerminate();
    i2C_deinit(fd);
}
