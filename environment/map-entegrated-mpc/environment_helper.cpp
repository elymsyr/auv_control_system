#include "environment.h"
#include <chrono>
#include <algorithm>
#include <tuple>
#include <stdio.h>
#include <cfloat>

float2 move_to(float x1, float y1, float x2, float y2, float factor) {
    // Calculate direction vector
    float dx = x2 - x1;
    float dy = y2 - y1;
    // Compute the distance to the target point
    float length = std::sqrt(dx * dx + dy * dy);
    // Compute new point
    float proj_x = x1 + (dx / length) * factor;
    float proj_y = y1 + (dy / length) * factor;
    return make_float2(proj_x, proj_y);
}

float distance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

float heuristic(int2 a, int2 b) {
    // Using Euclidean distance as heuristic
    return std::sqrt(static_cast<float>((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y)));
}