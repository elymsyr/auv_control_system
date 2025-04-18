#ifndef MISSION_SYSTEM_H
#define MISSION_SYSTEM_H

#include <functional>
#include <unordered_map>
#include "vehicle_model.h"
#include "environment_state.h"
// using TestMission = MissionTemplate<VehicleModel, EnvironmentState>;

template <typename VehicleModelType, typename EnvironmentStateType>
class MissionTemplate {
public:

    enum class MissionMode {
        WAIT,
        TEST,
        INIT,
        CONTROL,
        START,
        STOP,
        REPORT,
        END
    };

    struct MissionReport {
        MissionMode mission_mode = MissionMode::WAIT;
        bool success = false;
        std::unordered_map<std::string, bool> error_flags = {
            {"model_error", false},
            {"sensor_error", false},
            {"control_error", false},
            {"init_error", false},
            {"start_error", false},
            {"report_error", false},
        };

        void set_error(const std::string& error_type, bool value) {
            if (error_flags.find(error_type) != error_flags.end()) {
                error_flags[error_type] = value;
            }
        }

        bool has_error() const {
            for (const auto& [key, value] : error_flags) {
                if (value) return true;
            }
            return false;
        }
    };

    void set_mission(MissionMode mode);
    MissionMode check_mission() const;

private:
    MissionMode mission_mode = MissionMode::WAIT;
    VehicleModelType vehicle_model;

    void mission_step(EnvironmentStateType state);
    void mission_report();
    void mission_start();
    void mission_stop();
    void mission_init();
    void mission_test();
    void mission_control();
};

#endif // MISSION_SYSTEM_H