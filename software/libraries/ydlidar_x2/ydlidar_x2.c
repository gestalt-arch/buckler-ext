#include "ydlidar_x2.h"
#include <math.h>

#define DEG_TO_RAD 0.0174533
#define INDEX (i + index) % MAX_RESOLUTION

static lidar_struct data_struct;
static unsigned int index = 0;

void parse_distance(YdLidarData_t* lidar) {
    uint8_t* read_ptr = data_struct.Si;
    for (int i = 0; i < data_struct.LS; i++) {
        lidar->distance[INDEX] = (float)(read_ptr[1] << 8 | read_ptr[0]) / 4.f;
        read_ptr += 2;
    } 
}
 
void parse_theta(YdLidarData_t* lidar) {
    uint16_t FSA = (float)(data_struct.FSA_MSB << 8 | data_struct.FSA_LSB);
    uint16_t LSA = (float)(data_struct.LSA_MSB << 8 | data_struct.LSA_LSB);

    float theta_FSA = (FSA >> 1) / 64.f;
    float theta_LSA = (LSA >> 1) / 64.f;

    lidar->theta[(index) % MAX_RESOLUTION] = theta_FSA;
    lidar->theta[(index + data_struct.LS - 1) % MAX_RESOLUTION] = theta_LSA;

    float theta_dif = -theta_FSA + theta_LSA;

    for (int i = 2; i < data_struct.LS - 1; i++) {
        lidar->theta[INDEX] = theta_dif / (data_struct.LS - 1) * (i - 1) + theta_FSA;
    }

    for (int i = 0; i < data_struct.LS; i++) {
        float distance = lidar->distance[INDEX];
        lidar->theta[INDEX] += (distance == 0) ? 0 : atanf((21.8 * (155.3 - distance) / (155.3 * distance)) * DEG_TO_RAD);
    }
}

void parse_lidar(uint8_t* lidar_data, YdLidarData_t* lidar) {
    data_struct.PH_LSB = lidar_data[0];
    data_struct.PH_MSB = lidar_data[1];
    data_struct.CT = lidar_data[2];
    data_struct.LS = lidar_data[3];
    data_struct.FSA_LSB = lidar_data[4];
    data_struct.FSA_MSB = lidar_data[5];
    data_struct.LSA_LSB = lidar_data[6];
    data_struct.LSA_MSB = lidar_data[7];
    data_struct.CS_LSB = lidar_data[8];
    data_struct.CS_MSB = lidar_data[9];
    for (uint16_t i = 0; i < data_struct.LS * 2; i++) {
        data_struct.Si[i] = lidar_data[10 + i];
    }
}

void update_data(YdLidarData_t* lidar) {
    parse_distance(lidar);
    parse_theta(lidar);
    index += data_struct.LS;
}

void get_lidar_data(uint8_t* lidar_data, YdLidarData_t* lidar) {
    //u_int8_t lidar_data[522]; //= get_data from ic2
    parse_lidar(lidar_data, lidar);
    update_data(lidar);
}