## Sensors Simulation (MVP)

### Description
C++ Application that simulates sensors transmitting information with different
frequencies, and linearly interpolating values for slower sensors.

This solution has:
- A Sensor class, in charge of simulating double-valued sensor data and informing a SensorManager about it through a callback.
- Threads were used to simulate the ticks of the sensors and to have several of them running simultaneously.
- A SensorManager, configured as a Singleton, which is the only one capable of creating Sensors.
---

### Requirements

- CMake ≥ 3.30
- C++20 compatible compiler (I used the one from https://llvm.org/)

### Building the Project

#### Create build directory
- mkdir build
- cd build

#### Generate build files with CMake 
- cmake -DCMAKE_C_COMPILER=/path/to/clang \
-DCMAKE_CXX_COMPILER=/path/to/clang++ \
..

#### Build the executable
cmake --build .

#### Running the executable
./sensormanager
