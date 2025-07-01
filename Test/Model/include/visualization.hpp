#include <vector>
#include <csignal>
#include <atomic>
#include <sys/stat.h>
#include <GLFW/glfw3.h>
#include <thread>
#include <mutex>
#include <vector_types.h>
#include "environment.h"
#include <mutex>
#include <algorithm>
#include <casadi/casadi.hpp>

// Visualization data
struct VisualizationData {
    std::vector<float2> obstacles;
    std::vector<float2> path_points;
    std::vector<float> path_angles;
    float2 vehicle_pos = {0.0f, 0.0f};
    float vehicle_yaw = 0.0f;
    float2 goal_pos = {0.0f, 0.0f};
};

// Visualization constants
const int VIS_WIDTH = 800;
const int VIS_HEIGHT = 800;
float SAFETY_DIST;
VisualizationData vis_data;
std::mutex vis_mutex;
GLFWwindow* window = nullptr;

// Visualization functions
void draw_vehicle(float x, float y, float yaw) {
    glPushMatrix();
    glTranslatef(x, y, 0.0f);
    glRotatef(yaw * 180.0f / M_PI, 0.0f, 0.0f, 1.0f);
    
    // Draw vehicle body
    glColor3f(0.0f, 1.0f, 0.0f); // Green
    glBegin(GL_TRIANGLES);
    glVertex2f(0.5f, 0.0f);
    glVertex2f(-0.3f, 0.3f);
    glVertex2f(-0.3f, -0.3f);
    glEnd();
    
    // Draw heading indicator
    glColor3f(1.0f, 1.0f, 0.0f); // Yellow
    glBegin(GL_LINES);
    glVertex2f(0.0f, 0.0f);
    glVertex2f(0.5f, 0.0f);
    glEnd();
    
    glPopMatrix();
}

void draw_path(const std::vector<float2>& points) {
    if (points.empty()) return;
    
    glColor3f(0.0f, 0.0f, 1.0f); // Blue
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (const auto& pt : points) {
        glVertex2f(pt.x, pt.y);
    }
    glEnd();
}

void draw_goal(float x, float y) {
    glColor3f(1.0f, 0.0f, 1.0f); // Magenta
    glPointSize(10.0f);
    glBegin(GL_POINTS);
    glVertex2f(x, y);
    glEnd();
}

void draw_obstacles(const std::vector<float2>& obstacles) {
    if (obstacles.empty()) return;
    
    const int SEGMENTS = 36;
    
    glColor3f(1.0f, 0.0f, 0.0f);
    glLineWidth(0.5f);
    
    for (const auto& obs : obstacles) {
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < SEGMENTS; i++) {
            float theta = 2.0f * M_PI * float(i) / float(SEGMENTS);
            float dx = SAFETY_DIST * cosf(theta);
            float dy = SAFETY_DIST * sinf(theta);
            glVertex2f(obs.x + dx, obs.y + dy);
        }
        glEnd();
    }
    
    // Optional: Draw center points (keep if you want visible markers)
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    for (const auto& obs : obstacles) {
        glVertex2f(obs.x, obs.y);
    }
    glEnd();
}

void draw_orientation_indicators(const std::vector<float2>& points, const std::vector<float>& angles) {
    if (points.size() != angles.size()) return;
    
    glColor3f(0.0f, 1.0f, 1.0f);
    glLineWidth(1.5f);
    
    for (size_t i = 0; i < (int)points.size()/20; i++) {
        // Only draw every 5th indicator to avoid clutter
        if (i % 5 != 0) continue;
        
        float2 pt = points[i];
        float angle = angles[i];
        
        // Calculate direction vector
        float arrow_length = 1.0f;
        float2 dir = {
            arrow_length * cosf(angle),
            arrow_length * sinf(angle)
        };
        
        // Draw direction arrow
        glBegin(GL_LINES);
        glVertex2f(pt.x, pt.y);
        glVertex2f(pt.x + dir.x, pt.y + dir.y);
        glEnd();
        
        // Draw arrowhead
        float arrow_size = 0.3f;
        float2 perp = {-dir.y * arrow_size, dir.x * arrow_size};
        glBegin(GL_TRIANGLES);
        glVertex2f(pt.x + dir.x, pt.y + dir.y);
        glVertex2f(pt.x + dir.x * 0.7f + perp.x, pt.y + dir.y * 0.7f + perp.y);
        glVertex2f(pt.x + dir.x * 0.7f - perp.x, pt.y + dir.y * 0.7f - perp.y);
        glEnd();
    }
}

void render_visualization() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Lock mutex to safely access visualization data
    std::lock_guard<std::mutex> lock(vis_mutex);
    
    // Set up coordinate system
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float view_size = 25.0f;
    glOrtho(-view_size, view_size, -view_size, view_size, -1.0f, 1.0f);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // Draw coordinate grid
    glColor3f(0.3f, 0.3f, 0.3f); // Dark gray
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for (float i = -25.0f; i <= 25.0f; i += 5.0f) {
        // Vertical lines
        glVertex2f(i, -25.0f);
        glVertex2f(i, 25.0f);
        
        // Horizontal lines
        glVertex2f(-25.0f, i);
        glVertex2f(25.0f, i);
    }
    glEnd();
    
    // Draw elements
    draw_obstacles(vis_data.obstacles);
    draw_path(vis_data.path_points);
    draw_orientation_indicators(vis_data.path_points, vis_data.path_angles);
    draw_goal(vis_data.goal_pos.x, vis_data.goal_pos.y);
    draw_vehicle(vis_data.vehicle_pos.x, vis_data.vehicle_pos.y, vis_data.vehicle_yaw);
    
    glfwSwapBuffers(window);
}

void visualization_thread_func() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return;
    }
    
    window = glfwCreateWindow(VIS_WIDTH, VIS_HEIGHT, "MPC-A* Path Planning", NULL, NULL);
    if (!window) {
        glfwTerminate();
        std::cerr << "Failed to create GLFW window\n";
        return;
    }
    
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync
    
    // Set up OpenGL
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f); // Dark background
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    
    // Main visualization loop
    while (!shutdown_requested && !glfwWindowShouldClose(window)) {
        render_visualization();
        glfwPollEvents();
    }
    
    glfwDestroyWindow(window);
    glfwTerminate();
    window = nullptr;
}

float2 int2_to_float2(int2 pt, const EnvironmentMap& map) {
    float2 world;
    // Calculate world position considering map sliding
    world.x = map.world_position_.x + (pt.x - map.shift_total_.x) * map.r_m_;
    world.y = map.world_position_.y + (pt.y - map.shift_total_.y) * map.r_m_;
    return world;
}

// Modified visualization update to handle sliding
void update_visualization_data(const std::vector<float2>& new_obstacles,
                              const EnvironmentMap& map,
                              const casadi::DM& x0, 
                              const Path& path, const int N,
                              const float2& goal_pos) {
    std::lock_guard<std::mutex> lock(vis_mutex);
    
    // Update obstacles (only add new ones)
    for (const auto& obs : new_obstacles) {
        // Check if obstacle already exists
        bool exists = false;
        for (const auto& existing : vis_data.obstacles) {
            if (fabs(existing.x - obs.x) < 0.1f && fabs(existing.y - obs.y) < 0.1f) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            vis_data.obstacles.push_back(obs);
        }
    }
    
    // Update vehicle position
    vis_data.vehicle_pos.x = static_cast<double>(x0(0));
    vis_data.vehicle_pos.y = static_cast<double>(x0(1));
    vis_data.vehicle_yaw = static_cast<double>(x0(5));
    
    // Update goal position
    vis_data.goal_pos = goal_pos;
    
    // Convert path to world coordinates with sliding correction
    vis_data.path_points.clear();
    for (int k = 0; k < std::min(N, path.length); k++) {
        vis_data.path_points.push_back(path.trajectory[k]);
    }
    vis_data.path_points.clear();
    vis_data.path_angles.clear();
    for (int k = 0; k < std::min(N, path.length); k++) {
        vis_data.path_points.push_back(path.trajectory[k]);
        vis_data.path_angles.push_back(path.angle[k]);
    }
}