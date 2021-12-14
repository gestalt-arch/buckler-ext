#include "ydlidar_x2.h"
#include <math.h>

#define DEG_TO_RAD 0.0174533
#define INDEX (i + index) % RESOLUTION

static YdLidarData_t data;
static lidar_struct data_struct;
static unsigned int index = 0;

void parse_distance() {
    uint8_t* read_ptr = data_struct.Si;
    for (int i = 0; i < data_struct.LS; i++) {
        data.distance[INDEX] = (float)(read_ptr[1] << 8 | read_ptr[0]) / 4.f;
        read_ptr += 2 * sizeof(uint8_t);
    } 
}
 
void parse_theta() {
    uint16_t FSA = (float)(data_struct.FSA_MSB << 8 | data_struct.FSA_LSB);
    uint16_t LSA = (float)(data_struct.LSA_MSB << 8 | data_struct.LSA_LSB);

    float theta_FSA = (FSA >> 1) / 64;
    float theta_LSA = (LSA >> 1) / 64;

    data.theta[(index) % RESOLUTION] = theta_FSA;
    data.theta[(index + data_struct.LS - 1) % RESOLUTION] = theta_LSA;

    float theta_dif = theta_FSA - theta_LSA;

    for (int i = 2; i < data_struct.LS - 1; i++) {
        data.theta[INDEX] = theta_dif / (data_struct.LS - 1) * (i - 1) + theta_FSA;
    }

    for (int i = 0; i < data_struct.LS; i++) {
        float distance = data.distance[INDEX];
        data.theta[INDEX] += atanf((21.8 * (155.3 - distance) / (155.3 * distance)) * DEG_TO_RAD);
    }
}

void parse_lidar(uint8_t* lidar_data) {
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

void update_data() {
    parse_distance();
    parse_theta();
    index += data_struct.LS;
}

YdLidarData_t* get_lidar_data(uint8_t* lidar_data) {
    //u_int8_t lidar_data[522]; //= get_data from ic2
    parse_lidar(lidar_data);
    update_data();
    return &data;
}