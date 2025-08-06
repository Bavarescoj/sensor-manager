cmake_minimum_required(VERSION 3.30)

project(sensormanager)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

# Fetching nlohmann/json
FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.12.0
)
FetchContent_MakeAvailable(json)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
        ${CMAKE_SOURCE_DIR}/src/*.cpp
)

# Adding executable
add_executable(sensormanager ${SOURCES})

# Defining location of the data.json and includes
target_compile_definitions(sensormanager PRIVATE DATA_FILEPATH="${CMAKE_SOURCE_DIR}/data/data.json")
target_include_directories(sensormanager PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Linking nlohmann/json to target
target_link_libraries(sensormanager PRIVATE nlohmann_json::nlohmann_json)