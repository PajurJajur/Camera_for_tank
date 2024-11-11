#include <iostream> 
#include <stdlib.h>
#include <string>
#include <wiringPi.h>
#include <unistd.h>
#include <pigpio.h>
#include <wiringPiI2C.h> 
#include <cmath>
//Adres i status czujnika
#define BMI160_ADDRESS 0x69  
#define BMI160_STATUS_REG 0x1B
//Rejestry konfiguracji akcelerometru
#define ACC_CONF 0x40
#define ACC_RANGE 0x41
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

#define ACC_X_L 0x12
#define ACC_X_H 0x13
#define ACC_Y_L 0x14
#define ACC_Y_H 0x15
#define ACC_Z_L 0x16
#define ACC_Z_H 0x17


int i2C_init() {
    // Inicjalizacja I2C
    int fd = i2cOpen(1, BMI160_ADDRESS, 0);  
    if (fd < 0) {
        std::cerr << "Nie udało się zainicjować I2C" << std::endl;
        return -1;
    }
    i2cWriteByteData(fd, CMD, 0xB6);  
    gpioDelay(100000);  
    i2cWriteByteData(fd, CMD, 0xB0);
    gpioDelay(50000);               
    i2cWriteByteData(fd, CMD, 0x15);  
    gpioDelay(50000);
    i2cWriteByteData(fd, CMD, 0x11);
    gpioDelay(50000);                
    i2cWriteByteData(fd, GYRO_CONF, 0x08);  
    i2cWriteByteData(fd, GYRO_RANGE, 0x00);
    gpioDelay(50000); 
    i2cWriteByteData(fd, ACC_CONF, 0x08);  
    i2cWriteByteData(fd, ACC_RANGE, 0x03);                
    gpioDelay(50000);  // Czekamy 80 ms na zakończenie konfiguracji
    return fd;
}

void i2C_deinit(int fd1) {
    i2cClose(fd1);
}

void i2c_read_gyro(int fd ,float *angleX, float *angleY, float *angleZ, float dt, float alpha){
    
    uint8_t status = i2cReadByteData(fd, BMI160_STATUS_REG);
    std::cout << "Status: " << (int)status << std::endl;
if(status == 208){
    int16_t gyroX = (i2cReadByteData(fd, GYRO_X_H) << 8) | i2cReadByteData(fd, GYRO_X_L);
    int16_t gyroY = (i2cReadByteData(fd, GYRO_Y_H) << 8) | i2cReadByteData(fd, GYRO_Y_L);
    int16_t gyroZ = (i2cReadByteData(fd, GYRO_Z_H) << 8) | i2cReadByteData(fd, GYRO_Z_L);

    //int16_t accX = (i2cReadByteData(fd, ACC_X_H) << 8) | i2cReadByteData(fd, ACC_X_L);
    //int16_t accY = (i2cReadByteData(fd, ACC_Y_H) << 8) | i2cReadByteData(fd, ACC_Y_L);
    //int16_t accZ = (i2cReadByteData(fd, ACC_Z_H) << 8) | i2cReadByteData(fd, ACC_Z_L);

        // Przeliczanie wartości na stopnie na sekundę (dps) przy założeniu zakresu ±2000°/s
        float scale = 2000 / 32768.0;
        float gyroX_dps = gyroX * scale;
        float gyroY_dps = gyroY * scale;
        float gyroZ_dps = gyroZ * scale;

        //float acc_scale = 8.0 / 32768.0;
       // float accX_g = accX * acc_scale;
        //float accY_g = accY * acc_scale;
        //float accZ_g = accZ * acc_scale;
    
    //float angle_accX = atan2(accY_g, sqrt(accX_g * accX_g + accZ_g * accZ_g)) * 180 / M_PI;
    //float angle_accY = atan2(-accX_g, sqrt(accY_g * accY_g + accZ_g * accZ_g)) * 180 / M_PI;

    *angleX += gyroX_dps * dt;  
    *angleY += gyroY_dps * dt;  
    *angleZ += gyroZ_dps * dt;  

        if (*angleX >= 360.0) {
        *angleX -= 360.0;
        } else if (*angleX < 0) {
            *angleX += 360.0;
        }

        if (*angleY >= 360.0) {
            *angleY -= 360.0;
        } else if (*angleY < 0) {
            *angleY += 360.0;
        }

        if (*angleZ >= 360.0) {
            *angleZ -= 360.0;
        } else if (*angleZ < 0) {
            *angleZ += 360.0;
        }
        //std::cout << "ACCX: " << angle_accX << "ACCY: " << angle_accY << std::endl;
        std::cout << "AngleX: " << *angleX << "AngleY: " << *angleY << "AngleZ: " << *angleZ << std::endl;
}
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

