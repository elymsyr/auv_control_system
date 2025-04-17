#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#pragma once
#include <string>
#include "subdata.h"

struct SensorData : public BaseData {
    double depth;
    double temperature;
    double pressure;
    std::array<double, 6> nu;
    std::array<double, 6> nu_dot;

    SensorData() : depth(0.0), temperature(0.0) {}
};

#endif // SENSOR_DATA_H