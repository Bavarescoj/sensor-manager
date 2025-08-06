//
// Created by Juan Bavaresco on 31.07.25.
//

#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H
#include <vector>
#include <atomic>

class Sensor;

// Meyers Singleton for the Manager
class SensorManager {
public:

    static SensorManager& getInstance();
    //This method creates the sensors
    std::shared_ptr<Sensor> createSensor(double _frequency, double _min_signal, double _max_signal);
    SensorManager(const SensorManager&) = delete;
    SensorManager& operator=(const SensorManager&) = delete;
    // Starting and stopping the sensors
    void startSensors();
    void stop();
    // Writing to a JSON file
    bool writeJSON(const std::string& filename) const;

private:
    // Atomic flag for triggering all sensors at the same time
    const std::shared_ptr<std::atomic_flag> flag;

    // Mutex needed for when sensors use the callback
    std::mutex sensors_mutex;

    SensorManager();
    ~SensorManager();

    // For giving sensors an id
    std::atomic_uint current_id;
    // For holding the sensors
    std::vector<std::shared_ptr<Sensor>> sensors;

    /* Callback that sensors use to inform a tick happened
     * (Calculations could be done here if needed instead of at the end)
     */
    void sensorCallback(const std::shared_ptr<Sensor>& sensor);
};

#endif //SENSORMANAGER_H
