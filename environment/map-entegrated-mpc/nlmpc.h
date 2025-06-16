#ifndef NLMPC_H
#define NLMPC_H

#include <casadi/casadi.hpp>
#include <vector>
#include <optional>
#include <string>
#include "vehicle_model.h"

using namespace casadi;

class NonlinearMPC {
public:
    NonlinearMPC(const VehicleModel& model, int N = 40, double dt = 0.1);
    void reset_previous_solution();
    std::pair<DM, DM> solve(const DM& x0, const DM& x_ref, const DM& obs_positions);
    void set_obstacle_weight(double weight);
    void set_obstacle_epsilon(double epsilon);
    int num_obstacles_, N_;
private:
    VehicleModel model_;
    int nx_, nu_;
    double dt_, obstacle_weight_, obstacle_epsilon_, min_dist_;
    Opti opti_;
    MX X_, U_, eps_, obs_param_;
    MX x0_param_, x_ref_param_;
    std::optional<OptiSol> prev_sol_;

    void setup_optimization();
    MX barrier_function(const MX& position, const MX& obstacles);
    void avoidance_constraints(const MX& position, const MX& obstacles);
};

#endif // NLMPC_H