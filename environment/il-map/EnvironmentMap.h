// EnvironmentMap.h
#ifdef __cplusplus
extern "C" {
#endif

// Add these declarations
void* create_environment_map(int w, int h);
void destroy_environment_map(void* map);
void* create_point_batch(int count);
void destroy_point_batch(void* batch);

void launch_slide_kernel(void* map, int shiftX, int shiftY);
void launch_update_kernel(void* map, void* batch);

#ifdef __cplusplus
}
#endif