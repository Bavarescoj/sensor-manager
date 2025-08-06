#include <iostream>
#include "Sensor.h"

using namespace std;

int main() {
    // Creating two sensors
    SensorManager::getInstance().createSensor(1.0, 0.0, 50.0);
    SensorManager::getInstance().createSensor(100.0, 0.0, 50.0);

    // Starting the sensors
    SensorManager::getInstance().startSensors();

    // Letting it run for 2 seconds
    this_thread::sleep_for(2s);

    return 0;
}
