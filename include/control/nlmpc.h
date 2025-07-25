#ifndef NLMPC_H
#define NLMPC_H

#include <casadi/casadi.hpp>
#include <vector>
#include <optional>
#include <string>
#include "control/vehicle_model.h"

using namespace casadi;

class NonlinearMPC {
public:
    NonlinearMPC(const std::string& config_path, int N = 40, double dt = 0.1);
    void reset_previous_solution();
    std::pair<DM, DM> solve(const DM& x0, const DM& x_ref);
    void initialization();
    int num_obstacles_, N_;
private:
    VehicleModel model_;
    int nx_, nu_;
    double dt_;
    Opti opti_;
    MX X_, U_, eps_;
    MX x0_param_, x_ref_param_;
    std::optional<OptiSol> prev_sol_;

    void setup_optimization();
};

#endif // NLMPC_H