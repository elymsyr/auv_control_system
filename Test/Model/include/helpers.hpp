#include <random>
#include <iostream>
#include <atomic>

// Global state
std::atomic<bool> shutdown_requested(false);
std::random_device rd;
std::mt19937 gen(rd());

// Signal handler
void sigint_handler(int) {
    std::cout << "\nShutdown requested, finishing current work...\n";
    shutdown_requested = true;
}

// Helper functions
double rand_uniform(double min, double max) {
    std::uniform_real_distribution<double> dist(min, max);
    return dist(gen);
}

double rand_uniform_step(double min, double max, bool negate = true) {
    int n_steps = static_cast<int>((max - min) / 0.1);
    int idx = std::uniform_int_distribution<int>(0, n_steps)(gen);
    double val = min + idx * 0.1;
    return negate && rand() % 2 == 0 ? -val : val;
}

double rand_near(double abs_max) {
    return rand_uniform(-abs_max, abs_max);
}

DM generate_X_current() {
    DM eta = DM::vertcat({
        0, 0, 0,
        rand_near(0.05),
        rand_near(0.05),
        rand_near(M_PI)
    });

    DM nu = DM::vertcat({
        rand_near(1.0),
        rand_near(1.0),
        rand_near(0.5),
        rand_near(0.01),
        rand_near(0.01),
        rand_near(0.1)
    });

    return DM::vertcat({eta, nu});
}

std::vector<double> dm_to_vector(const DM& m) {
    std::vector<double> vec;
    for (casadi_int i = 0; i < m.size1(); ++i) {
        for (casadi_int j = 0; j < m.size2(); ++j) {
            vec.push_back(static_cast<double>(m(i, j)));
        }
    }
    return vec;
}

