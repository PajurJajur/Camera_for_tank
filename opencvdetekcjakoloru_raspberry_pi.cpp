#include <iostream>
#include <opencv2/opencv.hpp>
#include <wiringPi.h>
#include <unistd.h>
#include <cmath>

using namespace cv;
using namespace std;

// Definicje pinów PWM dla serw
const int SERVO_PIN_X = 12; // GPIO 12 (Pin 32)
const int SERVO_PIN_Y = 13; // GPIO 13 (Pin 33)

// Zakresy kątów dla serw
const int SERVO1_MIN_ANGLE = 180; // Min kąt serwa 1
const int SERVO1_MAX_ANGLE = 0;  // Max kąt serwa 1

const int SERVO2_MIN_ANGLE = 180; // Min kąt serwa 2
const int SERVO2_MAX_ANGLE = 90;  // Max kąt serwa 2

// Zakresy PWM dla serw
const int PWM_MIN = 20;  // Minimalna wartość PWM odpowiadająca 1 ms
const int PWM_MAX = 250; // Maksymalna wartość PWM odpowiadająca 2 ms

const int CHANGE_THRESHOLD = 50; // Próg zmiany 
const int SMOOTHING_FACTOR = 10; // Współczynnik wygładzania PWM

const float KP = 0.2;            // Współczynnik proporcjonalny
const float KI = 0.04;           // Współczynnik całkujący
const float KD = 0.02;           // Współczynnik derivacyjny

int map1(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Funkcja do konwersji pozycji na kąt w stopniach
int positionToAngle(int position, int center, int minAngle, int maxAngle) {
    return map1(position, 0, 2 * center, minAngle, maxAngle);
}

// Funkcja do konwersji kąta na sygnał PWM
int angleToPWM(int angle, int minAngle, int maxAngle, int minPWM, int maxPWM) {
    return map1(angle, minAngle, maxAngle, minPWM, maxPWM);


}
// Funkcja do wygładzania wartości PWM
int smoothPWM(int currentPWM, int previousPWM, int factor) {
    return previousPWM + (currentPWM - previousPWM) / factor;
}


float calculateDistance(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}
// Driver code
int main() {
    if (wiringPiSetupGpio() == -1) { // Użycie numeracji pinów GPIO
        cerr << "Nie można zainicjalizować WiringPi\n";
        return 1;
    }

    // Inicjalizacja pinów PWM
    pinMode(SERVO_PIN_X, PWM_OUTPUT);
    pinMode(SERVO_PIN_Y, PWM_OUTPUT);

    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(192);  // Ustawienie preskalera
    pwmSetRange(1024); // Ustawienie zakresu

    int hmin = 19, smin = 2, vmin = 176;
    int hmax = 94, smax = 155, vmax = 255;

    Mat image, image1, imagewykryty;

    namedWindow("paski_zmian", (640, 200));
    createTrackbar("Hue Min", "paski_zmian", &hmin, 179);
    createTrackbar("Saturation Min", "paski_zmian", &smin, 255);
    createTrackbar("Value Min", "paski_zmian", &vmin, 255);
    createTrackbar("Hue Max", "paski_zmian", &hmax, 179);
    createTrackbar("Saturation Max", "paski_zmian", &smax, 255);
    createTrackbar("Value Max", "paski_zmian", &vmax, 255);

    VideoCapture Cam(0);
    if (!Cam.isOpened()) {
        cerr << "Nie można otworzyć kamery\n";
        return 1;
    }

    namedWindow("camera", WINDOW_NORMAL);
    namedWindow("camera1", WINDOW_NORMAL);
    resizeWindow("camera", 800, 600);
    resizeWindow("camera1", 800, 600);
    
    int prevAngleX = (SERVO1_MIN_ANGLE + SERVO1_MAX_ANGLE) / 2;
    int prevAngleY = (SERVO2_MIN_ANGLE + SERVO2_MAX_ANGLE) / 2;
    int prevPWM_X = angleToPWM(prevAngleX, SERVO1_MIN_ANGLE, SERVO1_MAX_ANGLE, 50, 250);
    int prevPWM_Y = angleToPWM(prevAngleY, SERVO2_MIN_ANGLE, SERVO2_MAX_ANGLE, 250, 150);
    
    int prevErrorX = 0;
    int prevErrorY = 0;
    float integralX = 0;
    float integralY = 0;

    
    pwmWrite(SERVO_PIN_X, 150);
    pwmWrite(SERVO_PIN_Y, 175);
    
    while (true) {
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

            int angleX = KP * errorX + KI * integralX + KD * dErrorX;
            int angleY = KP * errorY + KI * integralY + KD * dErrorY;
            
            
            //int angleX = positionToAngle(posX, centerX, SERVO1_MIN_ANGLE, SERVO1_MAX_ANGLE);
            //int angleY = positionToAngle(posY, centerY, SERVO2_MIN_ANGLE, SERVO2_MAX_ANGLE);


            int currentPWM_X = angleToPWM(angleX, SERVO1_MIN_ANGLE, SERVO1_MAX_ANGLE, 250, 50);
            int currentPWM_Y = angleToPWM(angleY, SERVO2_MIN_ANGLE, SERVO2_MAX_ANGLE, 150, 210);
            
            // moze do wyrzucenia
            //int pwmX = angleToPWM(angleX, SERVO1_MIN_ANGLE, SERVO1_MAX_ANGLE, 100, 200);
            //int pwmY = angleToPWM(angleY, SERVO2_MIN_ANGLE, SERVO2_MAX_ANGLE, 250, 150);
            
            if(currentPWM_Y >210) currentPWM_Y =210;
            
            cout << "posX: " << posX << " posY: " << posY << endl;
            cout << "angleX: " << angleX << " angleY: " << angleY << endl;
            cout << "pwmX: " << currentPWM_X << " pwmY: " << currentPWM_Y << endl;
            cout << "CenterX: " << centerX << " CenterY: " << centerY << endl;
             cout << "distance: " << distance<<endl;
             
            if (distance >= CHANGE_THRESHOLD) {
                int smoothPWM_X = smoothPWM(currentPWM_X, prevPWM_X, SMOOTHING_FACTOR);
                int smoothPWM_Y = smoothPWM(currentPWM_Y, prevPWM_Y, SMOOTHING_FACTOR);
                cout << "smoothPWM_X: " << smoothPWM_X << " smoothPWM_Y " << smoothPWM_Y << endl;
                pwmWrite(SERVO_PIN_X, smoothPWM_X);
                pwmWrite(SERVO_PIN_Y, smoothPWM_Y);

                // Aktualizacja poprzednich kątów i PWM
                prevAngleX = angleX;
                prevAngleY = angleY;
                prevPWM_X = smoothPWM_X;
                prevPWM_Y = smoothPWM_Y;
                prevErrorX = errorX;
                prevErrorY = errorY;

                // Dodanie opóźnienia po zmianie wartości PWM
                usleep(20000); // 20 ms
                }
            
            
            
            ellipse(image, Point(posX, posY), Size(100, 100), 0, 0, 360, Scalar(255, 0, 0), 6);
            line(image, Point(posX - 100, posY), Point(posX + 100, posY), Scalar(255, 0, 0), 3);
            line(image, Point(posX, posY - 100), Point(posX, posY + 100), Scalar(255, 0, 0), 3);
        }

        imshow("camera", image);
        imshow("camera1", imagewykryty);
        if (waitKey(1) == 27) break;
}}
