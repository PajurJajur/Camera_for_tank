#include <iostream> 
#include <stdlib.h>
#include <string>
#include <wiringPi.h>
#include <unistd.h>
#include <pigpio.h>
#define PIN 4  // Pin GPIO do podłączenia czujnika

int main() {
    // Inicjalizacja pigpio
    if (gpioInitialise() < 0) {
        std::cerr << "Nie udało się zainicjować pigpio!" << std::endl;
        return -1;
    }

    while(true){

    // Ustawienie pinu jako wyjście, aby wysłać impuls „trigger”
    gpioSetMode(PIN, PI_OUTPUT);
    gpioWrite(PIN, PI_LOW);
    gpioDelay(2);  // Czekamy 2 mikrosekundy
    gpioWrite(PIN, PI_HIGH);
    gpioDelay(10);  // Impuls trwający 10 mikrosekund
    gpioWrite(PIN, PI_LOW);

    // Zmiana trybu na wejście, aby odebrać echo
    gpioSetMode(PIN, PI_INPUT);


    uint32_t startTick, endTick;

    while (gpioRead(PIN) == PI_LOW);
    startTick = gpioTick();

    while (gpioRead(PIN) == PI_HIGH);
    endTick = gpioTick();

    uint32_t pulseDuration = endTick - startTick;
    double distance = (pulseDuration * 0.0343) / 2;  // 0.0343 cm/μs to prędkość dźwięku

    std::cout << "Odległość: " << distance << " cm" << std::endl;

    sleep(1);
    }
    gpioTerminate();
    return 0;
}
