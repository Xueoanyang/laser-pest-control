#ifndef __UARTCV_H
#define __UARTCV_H
#include "stm32f4xx.h"  
#include "headfile.h"

// // 数据包结构体定义
// typedef struct {
//     int16_t x_coord;
//     int16_t y_coord;
// } CoordPacket;

typedef struct {
    int16_t obj_x ;      // 物体坐标 X
    int16_t obj_y ;      // 物体坐标 Y
    int16_t laser_x ;    // 激光点坐标 X
    int16_t laser_y ;    // 激光点坐标 Y
} CoordPacket;

typedef struct {
    uint32_t area_value; // 面积值
} AreaPacket;

void USART2_Init(void);
void ProcessCoordPacket(void);
void ProcessAreaPacket(void);

extern volatile uint8_t coord_ready;
void ParseCoordFrame(void);

extern uint8_t rx_buffer[6];
extern uint8_t rx_index;
extern CoordPacket current_coord_packet;
extern AreaPacket current_area_packet;
extern uint8_t packet_ready;

#endif


