#ifndef SERIAL_H
#define SERIAL_H
#include "main.h"


#define SEE_RX_BUF_SIZE 18		//DMA���ջ�������С
#define VISION_ATTACK 0x21
#define VISION_LOST 0x20

typedef struct{
	uint8_t status;
	union{
		uint8_t byte[4];
		fp32 value;
	}pitch, yaw;
}VisionData_t;

extern VisionData_t vision_data;//�ṹ���е����ݼ��ɻ�ȡ��̨�����Ƕ�

extern uint8_t rx_buf[SEE_RX_BUF_SIZE];		//DMA���ջ�����
extern uint8_t rx_data[SEE_RX_BUF_SIZE];


void getVisionData(uint8_t* data, VisionData_t* vision_data);
void uart6_tx(uint8_t *USART_RX_BUF , int len);

#endif
