#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include "subsystem.h"
#include "shared_data.h"
#include <boost/lockfree/spsc_queue.hpp>

class ControlSystem : public Subsystem {
private:
    boost::lockfree::spsc_queue<SignalData> commandQueue{50};

public:
    explicit ControlSystem(std::string name, int runtime, SystemData& system, int order, SharedSignalData& signalData);
    bool midInit() override;
    // void midSuspend() override;
    void midHalt() override;
    // void midResume() override;
    void liveLoop() override;
    SharedSignalData& signalData;
    void pushCommand(const SignalData& cmd);

private:
    void executeControlCommands(const SignalData& cmd);
};

#endif // CONTROL_SYSTEM_H