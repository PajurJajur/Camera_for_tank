#include <iostream> 
#include <stdlib.h>
#include <string>
#include <wiringPi.h>
#include <unistd.h>
#include <wiringPiI2C.h> 
#define BMI160_ADDRESS 0x69  // Adres I2C czujnika BMI160
#define BMI160_STATUS_REG 0x1B
// Rejestry konfiguracji żyroskopu
#define GYRO_CONF 0x42
#define GYRO_RANGE 0x43
#define CMD 0x7E

// Rejestry danych żyroskopu
#define GYRO_X_L 0x0C
#define GYRO_X_H 0x0D
#define GYRO_Y_L 0x0E
#define GYRO_Y_H 0x0F
#define GYRO_Z_L 0x10
#define GYRO_Z_H 0x11

float angleX = 0.0; 
float angleY = 0.0;
float angleZ = 0.0;
const float dt = 0.2; // Czas w sekundach (200 ms)

int main() {
    // Inicjalizacja I2C
    int fd = wiringPiI2CSetup(BMI160_ADDRESS);
    if (fd == -1) {
        std::cerr << "Nie udało się zainicjować I2C" << std::endl;
        return -1;
    }
    wiringPiI2CWriteReg8(fd, CMD, 0xB6);  
    usleep(50000);  // Czekamy 50 ms na inicjalizację
    // Konfiguracja żyroskopu
    wiringPiI2CWriteReg8(fd, CMD, 0x15); 
    usleep(300000);  

    wiringPiI2CWriteReg8(fd, GYRO_CONF, 0x0D);  
    wiringPiI2CWriteReg8(fd, GYRO_RANGE, 0x00); // Ustawienie zakresu ±250°/s

    usleep(80000);  // Czekamy 50 ms na inicjalizację

    // Odczyt danych z żyroskopu
    while (true) {

        uint8_t status = wiringPiI2CReadReg8(fd, BMI160_STATUS_REG);
        std::cout << "Status: " << (int)status << std::endl;


        int16_t gyroX = (wiringPiI2CReadReg8(fd, GYRO_X_H) << 8) | wiringPiI2CReadReg8(fd, GYRO_X_L);
        int16_t gyroY = (wiringPiI2CReadReg8(fd, GYRO_Y_H) << 8) | wiringPiI2CReadReg8(fd, GYRO_Y_L);
        int16_t gyroZ = (wiringPiI2CReadReg8(fd, GYRO_Z_H) << 8) | wiringPiI2CReadReg8(fd, GYRO_Z_L);

        // Przeliczanie wartości na stopnie na sekundę (dps) przy założeniu zakresu ±2000°/s
        float scale = 2000 / 32768.0;
        float gyroX_dps = gyroX * scale;
        float gyroY_dps = gyroY * scale;
        float gyroZ_dps = gyroZ * scale;

        angleX += gyroX_dps * dt; // Integrowanie prędkości kątowej
        angleY += gyroY_dps * dt;
        angleZ += gyroZ_dps * dt;

        std::cout << "Angle X: " << angleX << ", Angle Y: " << angleY << ", Angle Z: " << angleZ << std::endl;



        usleep(200000);  // Odświeżanie co 100 ms
    }

    return 0;
}