#include <stdbool.h>
#include <stdint.h>

#define MAX_SAMPLES 40
#define MAX_RESOLUTION 429
typedef struct {
    float theta[MAX_RESOLUTION];
    float distance[MAX_RESOLUTION];
} YdLidarData_t;

typedef struct {
    uint8_t PH_LSB;
    uint8_t PH_MSB;
    uint8_t CT;
    uint8_t LS;
    uint8_t FSA_LSB;
    uint8_t FSA_MSB;
    uint8_t LSA_LSB;
    uint8_t LSA_MSB;
    uint8_t CS_LSB;
    uint8_t CS_MSB;
    uint8_t Si[MAX_SAMPLES * 2];
} lidar_struct;

void parse_distance();
void parse_theta();
void parse_lidar(uint8_t* lidar_data, YdLidarData_t * lidar);
void update_data();

