#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include "shared_queue.h"
#include "sensor_data.h"
#include "signal_data.h"
#include <tuple>

class SharedGroundCommand 
  : public SharedQueue<uint32_t, 128>
{
public:
  // Override read to parse digits
  std::tuple<uint32_t,uint32_t,uint32_t,uint32_t,uint32_t> read() {
    auto opt = SharedQueue::read();
    if (!opt) return {9,9,9,9,9};

    uint32_t cmd = *opt;
    uint32_t d[5] = {9,9,9,9,9};
    for (int i = 4; i >= 0; --i) {
      d[i] = cmd % 10;
      cmd /= 10;
    }
    return {d[0],d[1],d[2],d[3],d[4]};
  }
};

#include "shared_queue.h"
#include "sensor_data.h"
#include "signal_data.h"

using SharedSignalData = SharedQueue<SignalData, 128>;

#endif // SHARED_DATA_H