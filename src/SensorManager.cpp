//
// Created by Juan Bavaresco on 31.07.25.
//

#include <iostream>
#include <fstream>
#include <map>
#include "SensorManager.h"
#include "Sensor.h"
#include <nlohmann/json.hpp>

/*
 * Initializing the Sensor Manager
 */
SensorManager::SensorManager():
flag(std::make_shared_for_overwrite<std::atomic_flag>()), current_id(0) {
    flag->clear();
}

/*
 * Destructor
 */
SensorManager::~SensorManager() {
    stop();
}

/*
 * For getting the singleton instance
 */
SensorManager& SensorManager::getInstance() {
    static SensorManager instance;
    return instance;
}

/*
 * createSensor()
 * Creates a sensor with frequency, limit values and an id
 */
std::shared_ptr<Sensor> SensorManager::createSensor(const double _frequency, const double _min_signal, const double _max_signal) {
    //make_shared requires a public constructor, so using new instead, how to improve?
    std::shared_ptr<Sensor> sensor(new Sensor(current_id++, _frequency, _min_signal, _max_signal));
    sensors.push_back(std::move(sensor));
    return sensors.back();
}

/*
 * startSensors()
 * Runs the sensors, sets the callback and starts the flag
 */
void SensorManager::startSensors() {
    // Run all sensors
    for(const std::shared_ptr<Sensor>& sensor : sensors) {
        sensor->run(flag, [this](const std::shared_ptr<Sensor>& ticked_sensor) {
        this->sensorCallback(ticked_sensor);});
    }

    // Change the flag and notify so all sensors start at the same time
    flag->test_and_set();
    flag->notify_all();
}

/*
 * sensorCallback()
 * Receives the sensor that ticked and adds the timestamp as buffered to the other sensors
 * (This should be improved so sensors who have that timestamp coming don't get it added
 * but didn't have enough time)
 */
void SensorManager::sensorCallback(const std::shared_ptr<Sensor>& ticked_sensor) {
    std::lock_guard<std::mutex> lock(sensors_mutex);

    // Add the latest timestamp as a buffered one to the other sensors
    for(const std::shared_ptr<Sensor>& sensor : sensors) {
        if (sensor->getId() != ticked_sensor->getId() &&
            ticked_sensor->getTimestamp() > sensor->getTimestamp()) {
            sensor->addBufferedData(Sensor::Sample(ticked_sensor->getTimestamp(), std::numeric_limits<double>::quiet_NaN(), true));
        }
    }
}

/*
 * writeJSON
 * For writing the combined data of the sensors into a JSON file
 */
bool SensorManager::writeJSON(const std::string& file_path) const {
    std::ofstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << file_path << std::endl;
        return false;
    }

    // Create a map to merge all samples by timestamp
    std::map<int64_t, std::map<unsigned int, std::pair<double, bool>>> timestamped_data;

    // Collect all samples organized by timestamp
    for (const auto& sensor : sensors) {
        std::vector<Sensor::Sample> combinedData = sensor->combineData();
        for (const auto& sample : combinedData) {
            int64_t timestamp = sample.timestamp.count();
            timestamped_data[timestamp][sensor->getId()] = {sample.value, sample.interpolated};
        }
    }

    // Building the JSON structure using an external library
    nlohmann::json root;
    // Trying out structured bindings
    for (const auto& [timestamp, sensors_data] : timestamped_data) {
        nlohmann::json sample_json;
        // Setting the timestamp
        sample_json["timestamp"] = timestamp;

        // Setting the sensor data
        nlohmann::json sensors_json;
        for (const auto& [sensor_id, value_pair] : sensors_data) {
            sensors_json[std::to_string(sensor_id)] = {
                {"value", value_pair.first},
                {"interpolated", value_pair.second}
            };
        }

        // Adding everything to the root
        sample_json["sensors"] = sensors_json;
        root["samples"].push_back(sample_json);
    }

    // Writing to the JSON file
    file << std::setw(2) << root << std::endl;
    file.close();

    std::cout << "Sensors data written to: " << file_path << std::endl;
    return true;
}

/*
 * stop()
 * Stops all the sensors
 */
void SensorManager::stop() {
    for(const std::shared_ptr<Sensor>& sensor : sensors) {
        sensor->stop();
    }

    // Error handling could be done with the result (but didn't have time)
    bool result = writeJSON(DATA_FILEPATH);

    sensors.clear();
}







