#include <iostream>
#include "../src/max30101/max30101.hpp"

int main() {
    MAX30101 sensor = MAX30101();

    sensor.verify();

    float temp = sensor.read_temperature();

    std::cout << "Temperature Reading: " << temp << " C" << std::endl;
}
