#ifndef SENSOR_SYSTEM_H
#define SENSOR_SYSTEM_H

#include "subsystem.h"
#include "shared_data.h"
#include <boost/lockfree/spsc_queue.hpp>

class SensorSystem {
private:
    boost::lockfree::spsc_queue<SensorData> queue{10};

public:
    explicit SensorSystem();

private:
    SensorData readEnvironmentSensors();
};

#endif // SENSOR_SYSTEM_H