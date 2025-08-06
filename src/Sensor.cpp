//
// Created by Juan Bavaresco on 29.07.25.
//

#include <iostream>
#include "Sensor.h"

/*
 * Initializing the static values
 */
std::mutex Sensor::generator_mutex;
std::random_device Sensor::random_seed;
std::mt19937 Sensor::random_generator(Sensor::random_seed());

/*
 * Constructor
 * It creates a duration based on the frequency for each sensor, sets the initial values
 * for the sensor's attributes and initializes the distribution for the generator
 */
Sensor::Sensor(const unsigned int _id, const double _frequency, const double _min_signal, const double _max_signal): id(_id),
        frequency(_frequency), tick_duration{std::chrono::duration<double>(1.0 / _frequency)},
        timestamp(std::chrono::milliseconds::zero()), distribution(_min_signal, _max_signal) {
    // Sets an initial value for the current signal
    updateSignal(std::chrono::milliseconds::zero());
}

/*
 * Destructor
 */
Sensor::~Sensor() {
    stop();
}

/*
 * Getters
 */
double Sensor::getFrequency() const {
    return frequency;
}

unsigned int Sensor::getId() const {
    return id;
}

Sensor::Sample Sensor::getCurrentSignal() const {
    return current_signal;
}

std::chrono::milliseconds Sensor::getTimestamp() const {
    return timestamp;
}

// Adds buffered data that needs to be linearly interpolated
void Sensor::addBufferedData(const Sample &buffered) {
    buffered_data.push_back(buffered);
}

/*
 * Sensor::run(const std::shared_ptr<std::atomic_flag>& flag)
 * Starts a thread that sleeps according to the frequency, and updates the sensor value
 * when it wakes up. Then, informs the SensorManager of the update, so that grouped
 * samples are outputted.
 */
void Sensor::run(const std::shared_ptr<std::atomic_flag>& flag, const std::function<void(std::shared_ptr<Sensor>)>& callback) {
    // Shared pointer from current sensor - will be used for the callback
    std::shared_ptr<Sensor> self = shared_from_this();

    // Starting a thread where the sensor will simulate sending information
    sensor_thread = std::jthread{[self, flag, callback](const std::stop_token& stoken) {
        // Waiting for start flag to be triggered, so all sensors start "simultaneously"
        flag->wait(false, std::memory_order_relaxed);
        // Start time of the thread and calculate sensor's next tick
        const std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        auto next_time = start_time + self->tick_duration;

        //We keep running until a stop is requested
        while (!stoken.stop_requested()) {
            // Sleeping the thread until next tick
            std::this_thread::sleep_until(next_time);
            // Locking the sensor's data
            {
                std::lock_guard<std::mutex> lock(self->sensor_mutex);
                // We get the current timestamp and update the signal
                self->timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                    next_time - start_time);
                self->updateSignal(self->timestamp);
            }
            // We inform the SensorManager and calculate the next tick
            callback(self);
            next_time += self->tick_duration;
        }
    }};
}

/*
 * stop()
 * Stop each sensor's thread
 */
void Sensor::stop() {
    // Requesting a stop and joining the thread (if it's joinable of course)
    if (sensor_thread.joinable()) {
        sensor_thread.request_stop();
        sensor_thread.join();
    }
}

/*
 * linearInterpolation()
 * Uses the two known samples in between the timestamp to calculate the value
 */
double Sensor::linearInterpolation(const Sample &past, const Sample &current,
        const std::chrono::milliseconds timestamp) {

    // Duration between the two samples
    const double totalDuration = static_cast<double>((current.timestamp - past.timestamp).count());
    // The duration where we want to know the value
    const double targetDuration = static_cast<double>((timestamp - past.timestamp).count());
    // Interpolating the value linearly
    const double interpolated_value = past.value +
        (targetDuration * ((current.value - past.value) / totalDuration));
    return interpolated_value;
}

/*
 * updateSignal()
 * Generates a new value for the sensor, reassigns the values for the current and
 * previous values of the sensor, and return the current value.
 */
Sensor::Sample Sensor::updateSignal(const std::chrono::milliseconds timestamp) {
    // Locking the common generator
    std::lock_guard<std::mutex> lock(generator_mutex);
    current_signal = Sensor::Sample(timestamp, distribution(random_generator), false);
    sensor_data.push_back(current_signal);
    return current_signal;
}

/*
 * combineData()
 * It merges the real data from the sensor, with the interpolated values calculated
 * with the buffered timestamps
 */
std::vector<Sensor::Sample> Sensor::combineData() {

    // This will hold the resulting combined data
    std::vector<Sample> result;
    result.reserve(sensor_data.size() + buffered_data.size());

    // iterators indexes for going through both "real" and buffered data
    size_t i = 0, j = 0;

    // Putting the data by timestamp
    while (i < sensor_data.size() && j < buffered_data.size()) {
        // Adding first the "real" data
        if (sensor_data[i].timestamp < buffered_data[j].timestamp) {
            result.push_back(sensor_data[i++]);
        } else if (sensor_data[i].timestamp == buffered_data[j].timestamp) {
            result.push_back(sensor_data[i++]);
            j++;
        } else {
            // Calculating the linearly interpolated data
            buffered_data[j].value = linearInterpolation(sensor_data[i - 1],
            sensor_data[i], buffered_data[j].timestamp);
            result.push_back(buffered_data[j++]);
        }
    }

    // Add remaining elements
    while (i < sensor_data.size()) result.push_back(sensor_data[i++]);
    while (j < buffered_data.size()) {
        buffered_data[j].value = sensor_data.back().value;
        result.push_back(buffered_data[j++]);
    }

    return result;
}






