#ifndef SIGNAL_DATA_H
#define SIGNAL_DATA_H

#include "subdata.h"
#include <array>

struct SignalData : public BaseData {
    double propeller[6];
    double lighting;
    double system;

    SignalData(int id = 0) : BaseData(id), lighting(0.0), system(0.0) {
        for (double &p : propeller) {
            p = 0.0;
        }
    }
};
#endif // SIGNAL_DATA_H