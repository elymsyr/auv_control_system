#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cstdint>

enum CellCode : uint8_t {
  UNDISCOVERED   = 0,
  DISCOVERED     = 1,
  OBSTACLE       = 2,
  LINE_SEGMENT   = 3,
  OBJECT_UNDETECTED = 4,
  OBJECT_DETECTED   = 5,
  UNKNOWN        = 6
};

struct Obstacle {
  float distance_cm;
  float azimuth_rad;
  float elevation_rad;
  float relativeSpeed;
  uint8_t size_code;
  uint8_t semantic;
};

struct SonarBeam {
  float azimuth_rad;
  float elevation_rad;
  float distance_cm;
};

struct Detection {
  float x_cm, y_cm, z_cm;
  uint8_t type;
};

class LocalOccupancyMap {
public:
  static const int NX = 129, NY = 129, NZ = 33;
  static const int CX = NX/2, CY = NY/2, CZ = NZ/2;
  static const float DX, DY, DZ;

  LocalOccupancyMap() {
    memset(grid, 0, sizeof(grid));
    x_cm = y_cm = z_cm = 0.0f;
    yaw_rad = 0.0f;
    prev_x_cm = prev_y_cm = prev_z_cm = 0.0f;
  }

  void setPose(float x, float y, float z, float yaw) {
    x_cm = x;
    y_cm = y;
    z_cm = z;
    yaw_rad = yaw;
  }

  void iterate() {
    // Calculate voxel shifts
    float dx = x_cm - prev_x_cm;
    float dy = y_cm - prev_y_cm;
    float dz = z_cm - prev_z_cm;
    
    int sx = static_cast<int>(std::floor(dx / DX));
    int sy = static_cast<int>(std::floor(dy / DY));
    int sz = static_cast<int>(std::floor(dz / DZ));

    if (sx != 0 || sy != 0 || sz != 0) {
      slideGrid(sx, sy, sz);
      x_cm -= sx * DX;
      y_cm -= sy * DY;
      z_cm -= sz * DZ;
    }

    // Update previous pose
    prev_x_cm = x_cm;
    prev_y_cm = y_cm;
    prev_z_cm = z_cm;

    // Process sensor data
    processSonar();
    processCamera();
    
    // Update obstacles
    updateObstacleList();
  }

  void integrateSonar(const std::vector<SonarBeam>& beams) {
    sonarBuffer.insert(sonarBuffer.end(), beams.begin(), beams.end());
  }

  void integrateCamera(const std::vector<Detection>& dets) {
    cameraBuffer.insert(cameraBuffer.end(), dets.begin(), dets.end());
  }

  const std::vector<Obstacle>& getTopKThreats(size_t K) {
    if (K < obstacleList.size()) {
      obstacleList.resize(K);
    }
    return obstacleList;
  }

  uint8_t getCell(int i, int j, int k) const { return grid[i][j][k]; }

private:
  float x_cm, y_cm, z_cm;
  float yaw_rad;
  float prev_x_cm, prev_y_cm, prev_z_cm;
  uint8_t grid[NX][NY][NZ];
  uint8_t tempGrid[NX][NY][NZ];
  std::vector<Obstacle> obstacleList;
  std::vector<SonarBeam> sonarBuffer;
  std::vector<Detection> cameraBuffer;

  void slideGrid(int sx, int sy, int sz) {
    memset(tempGrid, 0, sizeof(tempGrid));

    for (int z = 0; z < NZ; ++z) {
      int oz = z - sz;
      if (oz < 0 || oz >= NZ) continue;
      
      for (int y = 0; y < NY; ++y) {
        int oy = y - sy;
        if (oy < 0 || oy >= NY) continue;
        
        for (int x = 0; x < NX; ++x) {
          int ox = x - sx;
          if (ox >= 0 && ox < NX) {
            tempGrid[x][y][z] = grid[ox][oy][oz];
          }
        }
      }
    }
    
    memcpy(grid, tempGrid, sizeof(grid));
  }

  void processSonar() {
    for (const auto& beam : sonarBuffer) {
      // Simplified ray casting - implement proper 3D DDA here
      int x1 = CX + static_cast<int>((beam.distance_cm * cos(beam.elevation_rad) * cos(beam.azimuth_rad)) / DX);
      int y1 = CY + static_cast<int>((beam.distance_cm * cos(beam.elevation_rad) * sin(beam.azimuth_rad)) / DY);
      int z1 = CZ + static_cast<int>((beam.distance_cm * sin(beam.elevation_rad)) / DZ);
      
      markRay(CX, CY, CZ, x1, y1, z1);
      if (x1 >= 0 && x1 < NX && y1 >= 0 && y1 < NY && z1 >= 0 && z1 < NZ) {
        grid[x1][y1][z1] = OBSTACLE;
      }
    }
    sonarBuffer.clear();
  }

  void markRay(int x0, int y0, int z0, int x1, int y1, int z1) {
    // Implement proper 3D ray casting algorithm
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int dz = abs(z1 - z0), sz = z0 < z1 ? 1 : -1;
    
    int dm = std::max({dx, dy, dz});
    dx *= 2, dy *= 2, dz *= 2;
    
    for (int i = 0; i < dm; ++i) {
      if (x0 >= 0 && x0 < NX && y0 >= 0 && y0 < NY && z0 >= 0 && z0 < NZ) {
        if (grid[x0][y0][z0] == UNDISCOVERED) {
          grid[x0][y0][z0] = DISCOVERED;
        }
      }
      
      if (dx >= dm) { x0 += sx; dx -= 2*dm; }
      if (dy >= dm) { y0 += sy; dy -= 2*dm; }
      if (dz >= dm) { z0 += sz; dz -= 2*dm; }
    }
  }

  void processCamera() {
    for (const auto& det : cameraBuffer) {
      int x = static_cast<int>((det.x_cm - (x_cm - CX*DX)) / DX);
      int y = static_cast<int>((det.y_cm - (y_cm - CY*DY)) / DY);
      int z = static_cast<int>((det.z_cm - (z_cm - CZ*DZ)) / DZ);
      
      if (x >= 0 && x < NX && y >= 0 && y < NY && z >= 0 && z < NZ) {
        grid[x][y][z] = (det.type == 0) ? OBJECT_DETECTED : LINE_SEGMENT;
      }
    }
    cameraBuffer.clear();
  }

  void updateObstacleList() {
    obstacleList.clear();
    
    for (int z = 0; z < NZ; ++z) {
      for (int y = 0; y < NY; ++y) {
        for (int x = 0; x < NX; ++x) {
          if (grid[x][y][z] >= OBSTACLE && grid[x][y][z] <= OBJECT_DETECTED) {
            float px = (x_cm - CX*DX) + (x + 0.5f)*DX;
            float py = (y_cm - CY*DY) + (y + 0.5f)*DY;
            float pz = (z_cm - CZ*DZ) + (z + 0.5f)*DZ;
            
            float dx = px - x_cm;
            float dy = py - y_cm;
            float dz = pz - z_cm;
            
            Obstacle obs {
              std::hypot(dx, dy, dz),
              std::atan2(dy, dx),
              std::atan2(dz, std::hypot(dx, dy)),
              0.0f,  // Placeholder for speed
              static_cast<uint8_t>(1),  // Size code
              grid[x][y][z]
            };
            
            obstacleList.push_back(obs);
          }
        }
      }
    }
    
    std::sort(obstacleList.begin(), obstacleList.end(),
      [](const Obstacle& a, const Obstacle& b) { 
        return a.distance_cm < b.distance_cm; 
      });
  }
};

const float LocalOccupancyMap::DX = 25.0f;
const float LocalOccupancyMap::DY = 25.0f;
const float LocalOccupancyMap::DZ = 50.0f;

#include <iostream>
#include <thread>
#include <chrono>

constexpr int VIS_SIZE = 21;  // Half of 41x41 visualization window

void visualizeXY(const LocalOccupancyMap& map, int z) {
    std::cout << "\nXY Slice at z=" << z << ":\n";
    // Use map.CY instead of CY
    for(int y = map.CY-VIS_SIZE/2; y < map.CY+VIS_SIZE/2; ++y) {
        for(int x = map.CX-VIS_SIZE/2; x < map.CX+VIS_SIZE/2; ++x) {
            char c = '.';
            if(x == map.CX && y == map.CY) c = 'V';  // Vehicle position
            else if(x >= 0 && x < map.NX && y >= 0 && y < map.NY) {
                switch(map.getCell(x, y, z)) {
                    case OBSTACLE: c = 'O'; break;
                    case DISCOVERED: c = ' '; break;
                    case OBJECT_DETECTED: c = '*'; break;
                    case LINE_SEGMENT: c = '-'; break;
                }
            }
            std::cout << c << ' ';
        }
        std::cout << '\n';
    }
}

void visualizeXZ(const LocalOccupancyMap& map) {
    std::cout << "\nXZ Slice at y=" << map.CY << ":\n";
    for(int z = map.NZ-1; z >= 0; --z) {
        for(int x = map.CX-VIS_SIZE/2; x < map.CX+VIS_SIZE/2; ++x) {
            char c = '.';
            if(x == map.CX && z == map.CZ) c = 'V';
            else if(x >= 0 && x < map.NX && z >= 0 && z < map.NZ) {
                switch(map.getCell(x, map.CY, z)) {
                    case OBSTACLE: c = 'O'; break;
                    case DISCOVERED: c = ' '; break;
                    case OBJECT_DETECTED: c = '*'; break;
                }
            }
            std::cout << c << ' ';
        }
        std::cout << " z=" << z << '\n';
    }
}

int main() {
    LocalOccupancyMap map;
    
    // Test scenario - vehicle moving in a square pattern
    for(int step = 0; step < 50; ++step) {
        std::cout << "\n\n=== STEP " << step << " ===\n";
        
        // Simulate vehicle movement (25cm per step in X direction)
        float x = step * 25.0f;
        float y = 0.0f;
        map.setPose(x, y, 0.0f, 0.0f);

        // Generate test sensor data
        std::vector<SonarBeam> beams = {
            {0.0f, 0.0f, 200.0f},  // Forward beam
            {M_PI/4, 0.0f, 150.0f} // Diagonal beam
        };
        
        std::vector<Detection> detections;
        if(step % 10 == 0) {  // Periodic object detection
            detections.push_back({x + 300.0f, y + 100.0f, 0.0f, OBJECT_DETECTED});
        }

        // Integrate sensors and update
        map.integrateSonar(beams);
        map.integrateCamera(detections);
        map.iterate();

        // Visualize
        visualizeXY(map, map.CZ);    // Horizontal slice at vehicle height
        visualizeXZ(map);            // Vertical slice through vehicle

        // Show threats
        const auto& threats = map.getTopKThreats(3);
        std::cout << "\nTop Threats:\n";
        for(const auto& t : threats) {
            std::cout << "Dist: " << t.distance_cm << "cm \t" 
                      << "Az: " << t.azimuth_rad << "rad \t"
                      << "Type: " << (int)t.semantic << '\n';
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}