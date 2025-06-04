#include <vector>
#include <cmath>

class EnvironmentIntegrator {
public:
    EnvironmentIntegrator(int width, int height, float resolution)
        : width_(width), height_(height), resolution_(resolution) {}
    
    // Update environment data (call this whenever the environment changes)
    void updateEnvironment(const std::vector<uint8_t>& grid_data) {
        grid_ = grid_data;
    }

    // Calculate barrier function h(x) for a given position
    double calculate_h(double x, double y) const {
        // Convert world coordinates to grid indices
        int grid_x = static_cast<int>((x + (width_ * resolution_)/2) / resolution_);
        int grid_y = static_cast<int>((y + (height_ * resolution_)/2) / resolution_);

        // Check bounds
        if (grid_x < 0 || grid_x >= width_ || grid_y < 0 || grid_y >= height_) {
            return -1.0;  // Consider out-of-bounds as unsafe
        }

        // Get grid value
        uint8_t cell_value = grid_[grid_y * width_ + grid_x];
        
        // Calculate barrier function (higher = safer)
        // Customize this based on your cell encoding
        double h_value;
        switch (cell_value) {
            case OBSTACLE:
                h_value = -10.0;  // Very unsafe
                break;
            case UNDISCOVERED:
                h_value = 0.5;    // Moderately safe
                break;
            case FREE_SPACE:
                h_value = 1.0;    // Safe
                break;
            default:
                h_value = 0.0;    // Neutral
        }
        
        return h_value;
    }

    // Calculate gradient of h(x) (finite differences)
    std::pair<double, double> calculate_h_gradient(double x, double y, double eps = 0.01) const {
        double h_val = calculate_h(x, y);
        double hx_plus = calculate_h(x + eps, y);
        double hy_plus = calculate_h(x, y + eps);
        
        double dh_dx = (hx_plus - h_val) / eps;
        double dh_dy = (hy_plus - h_val) / eps;
        
        return {dh_dx, dh_dy};
    }

private:
    enum CellState {
        UNDISCOVERED = 0,
        FREE_SPACE = 1,
        OBSTACLE = 2,
        // Add other states as needed
    };
    
    int width_;
    int height_;
    float resolution_;
    std::vector<uint8_t> grid_;
};