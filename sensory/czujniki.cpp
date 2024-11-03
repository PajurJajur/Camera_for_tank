#include <iostream> 
#include <stdlib.h>
#include <string>
#include <wiringPi.h>
#include <unistd.h>
#include <pigpio.h>
#include <wiringPiI2C.h> 

//Adres i status czujnika
#define BMI160_ADDRESS 0x69  
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


int i2C_init(){
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
    return fd;

}

void i2c_read_gyro(int fd ,float *angleX, float *angleY, float *angleZ, float dt){
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

    if (*angleX > 360.0f) {
        *angleX -= 360.0f;
    } else if (*angleX < -360.0f) {
        *angleX += 360.0f;
    }

    if (*angleY > 360.0f) {
        *angleY -= 360.0f;
    } else if (*angleY < -360.0f) {
        *angleY += 360.0f;
    }

    if (*angleZ > 360.0f) {
        *angleZ -= 360.0f;
    } else if (*angleZ < -360.0f) {
        *angleZ += 360.0f;
    }

        *angleX += gyroX_dps * dt; 
        *angleY += gyroY_dps * dt;
        *angleZ += gyroZ_dps * dt;
}


double distance_sensor_measure(int PIN) {
    // Ustawienie pinu jako wyjście, aby wysłać impuls „trigger”
    gpioSetMode(PIN, PI_OUTPUT);
    gpioWrite(PIN, PI_LOW);
    gpioDelay(2);  // Czekamy 2 mikrosekundy
    gpioTrigger(PIN, 10, PI_HIGH);  // 10-mikrosekundowy impuls triggera

    // Zmiana trybu na wejście, aby odebrać echo
    gpioSetMode(PIN, PI_INPUT);

    uint32_t startTick, endTick;
    uint32_t timeout = 30000;  // 30 ms timeout (dla maksymalnej odległości ~500 cm)

    // Czekaj na sygnał HIGH
    uint32_t startWait = gpioTick();
    while (gpioRead(PIN) == PI_LOW) {
        if (gpioTick() - startWait > timeout) {
            std::cerr << "Timeout waiting for HIGH signal" << std::endl;
            return -1;  // Timeout — brak sygnału HIGH
        }
    }
    startTick = gpioTick();

    // Czekaj na sygnał LOW
    startWait = gpioTick();
    while (gpioRead(PIN) == PI_HIGH) {
        if (gpioTick() - startWait > timeout) {
            std::cerr << "Timeout waiting for LOW signal" << std::endl;
            return -1;  // Timeout — brak sygnału LOW
        }
    }
    endTick = gpioTick();

    uint32_t pulseDuration = endTick - startTick;

    double distance = (pulseDuration * 0.0343) / 2;  // Przeliczenie na cm
    return distance;
}

