#ifndef SENSOR_SYSTEM_H
#define SENSOR_SYSTEM_H

#include "subsystem.h"
#include "shared_data.h"
#include <boost/lockfree/spsc_queue.hpp>

class SensorSystem : public Subsystem {
private:
    SharedSensorData& sharedData;
    boost::lockfree::spsc_queue<SensorData> queue{100};

public:
    explicit SensorSystem(std::string name, int runtime, SystemData& system, int order, SharedSensorData& sd);
    bool midInit() override;
    // void midSuspend() override;
    // void midHalt() override;
    // void midResume() override;
    void liveLoop() override;

private:
    SensorData readHardwareSensors();
};

#endif // SENSOR_SYSTEM_H