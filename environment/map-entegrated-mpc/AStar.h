#include <cmath>
#include <vector>
#include <cuda_runtime.h>
#include <cstdint>

struct Node {
    int x, y;
    float g, h, f;
    int parent_x, parent_y;
    int status;
};

struct Path {
    int2* points; // device pointer to path points
    int length;   // number of points in path
};

class AStar {
public:
    AStar(int width, int height);
    ~AStar();

    void setGoal(int x, int y);
    void updateGrid(uint8_t* gridData);
    Path findPath(uint8_t* grid_);
    void freePath(Path path);

private:
    int width_, height_;
    int start_x_, start_y_;
    int goal_x_, goal_y_;
    Node* d_grid_;

    void initializeGrid();
};