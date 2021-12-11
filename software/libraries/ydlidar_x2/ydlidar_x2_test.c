#include "ydlidar_x2.h"
#include <stdio.h>

int main() {
    uint8_t buffer[20] = {0xAA, 0xAA, 0x01, 0x05, 0xE5, 0x6F, 0xBD, 0x79, 0xA0, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    YdLidarData_t* bob = get_lidar_data(buffer);

    printf("Distance: ");
    for (int i = 0; i < 5; i++) {
        printf("%f, ", bob->distance[i]);
    }

    printf("\nTheta: ");
    for (int i = 0; i < 5; i++) {
        printf("%f, ", bob->theta[i]);
    }

}