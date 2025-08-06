//
// Created by Juan Bavaresco on 29.07.25.
//

#ifndef SENSOR_H
#define SENSOR_H

#include <chrono>
#include <thread>
#include <atomic>
#include <random>
#include "SensorManager.h"

class Sensor : public std::enable_shared_from_this<Sensor> {
public:
    ~Sensor();

    // Struct for each sample data from the sensors
    struct Sample {
        std::chrono::milliseconds timestamp;
        double value;
        bool interpolated;

        // Overloading << operator for the sample struct
        friend std::ostream& operator<<(std::ostream& os, const Sample& sample) {
            os << "Sample = " << sample.timestamp << ", " << sample.value << "";
            return os;
        }
    };

    // Getters
    double getFrequency() const;
    unsigned int getId() const;
    Sample getCurrentSignal() const;
    std::chrono::milliseconds getTimestamp() const;

    // Adds a buffered data sample
    void addBufferedData(const Sample &buffered);

    // Starting and running the sensor in its own thread
    void run(const std::shared_ptr<std::atomic_flag>& flag, const std::function<void(std::shared_ptr<Sensor>)>& callback);
    // Stops the sensor
    void stop();

    // Calculates the linear interpolation (should this be on the SensorManager?)
    static double linearInterpolation(const Sample &past, const Sample &current,
                                      std::chrono::milliseconds timestamp);

    // Method for combining both "real" and interpolated data
    std::vector<Sample> combineData();

private:
    // Private constructor - only the SensorManager can do it
    Sensor(unsigned int _id, double _frequency, double _min_signal, double _max_signal);
    // Simulating the signal
    Sample updateSignal(std::chrono::milliseconds);

    // ID of the sensor
    unsigned int id;
    // Frequency of the sensor
    double frequency;
    const std::chrono::duration<double, std::milli> tick_duration;
    Sample current_signal{};
    std::chrono::milliseconds timestamp;
    std::mutex sensor_mutex;

    // Data (both concrete and linearly interpolated) from the sensor
    std::vector<Sample> sensor_data;
    // Buffered timestamps for which linear interpolation will be needed
    std::vector<Sample> buffered_data;

    // Thread for simulating ticks
    std::jthread sensor_thread;

    // Random number generator with uniform distribution
    static std::random_device random_seed;
    static std::mt19937 random_generator;
    static std::mutex generator_mutex;
    std::uniform_real_distribution<> distribution;

    // Only the SensorManager can create sensors (or in a non-simulated case, register them?)
    friend std::shared_ptr<Sensor> SensorManager::createSensor(double _frequency, double _min_signal, double _max_signal);
};

#endif //SENSOR_H
