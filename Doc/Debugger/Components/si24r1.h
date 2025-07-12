#pragma once

#include "system.h"
#include "spi.h"

#define SI24R1_SPI_HANDLE hspi1
#define SI24R1_CHECK_MAX  5

// 默认自动重发尝试次数
#define REPEAT_CNT          15
// 默认发送地址和接收地址
#define DEFAULT_TX_ADDR	    0x34,0x43,0x10,0x10,0x01
#define DEFAULT_RX_ADDR	    0x34,0x43,0x10,0x10,0x01

// CE 引脚操作宏
#define SI24R1_CE_HIGH()  	HAL_GPIO_WritePin(G01_CE_GPIO_Port, G01_CE_Pin, GPIO_PIN_SET)
#define SI24R1_CE_LOW()   	HAL_GPIO_WritePin(G01_CE_GPIO_Port, G01_CE_Pin, GPIO_PIN_RESET)
// CS 引脚操作宏
#define SI24R1_CS_HIGH()	HAL_GPIO_WritePin(G01_CS_GPIO_Port,G01_CS_Pin, GPIO_PIN_SET)
#define SI24R1_CS_LOW()		HAL_GPIO_WritePin(G01_CS_GPIO_Port,G01_CS_Pin, GPIO_PIN_RESET)
// 获取G01 IRQ引脚电平
#define SI24R1_GET_IRQ_PIN()		HAL_GPIO_ReadPin(G01_IRQ_GPIO_Port, G01_IRQ_Pin)

// 设置SI24R1获取中断的方式(读取IRQ引脚或读取STATUS寄存器)
#define SI24R1_GET_IRQ_STATUS()     SI24R1_GET_IRQ_PIN()
// #define RF24L01_GET_IRQ_STATUS( )		(( RF24L01_IRQ_GPIO_PORT->IDR & (uint32_t)RF24L01_IRQ_GPIO_PIN) != RF24L01_IRQ_GPIO_PIN ) ? 0 : 1	//IRQ状态

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Types
typedef enum ModeType
{
	MODE_TX = 0,
	MODE_RX
}si24r1_mode_t;

typedef enum SpeedType
{
	SPEED_250K = 0,
	SPEED_1M,
	SPEED_2M
}si24r1_speed_type;
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Commands
#define NRF_READ_REG    0x00	//读配置寄存器，低5位为寄存器地址
#define NRF_WRITE_REG   0x20	//写配置寄存器，低5位为寄存器地址
#define RD_RX_PLOAD     0x61	//读RX有效数据，1~32字节
#define WR_TX_PLOAD     0xA0	//写TX有效数据，1~32字节
#define FLUSH_TX        0xE1	//清除TX FIFO寄存器，发射模式下使用
#define FLUSH_RX        0xE2	//清除RX FIFO寄存器，接收模式下使用
#define REUSE_TX_PL     0xE3	//重新使用上一包数据，CE为高，数据包被不断发送
#define R_RX_PL_WID     0x60
#define NOP             0xFF	//空操作，可以用来读状态寄存器
#define W_ACK_PLOAD		0xA8
#define WR_TX_PLOAD_NACK 0xB0
// Registers
#define CONFIG          0x00	//配置寄存器地址，bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
							    //bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能	
#define EN_AA           0x01	//使能自动应答功能 bit0~5 对应通道0~5
#define EN_RXADDR       0x02	//接收地址允许 bit0~5 对应通道0~5
#define SETUP_AW        0x03	//设置地址宽度(所有数据通道) bit0~1: 00,3字节 01,4字节, 02,5字节
#define SETUP_RETR      0x04	//建立自动重发;bit0~3:自动重发计数器;bit4~7:自动重发延时 250*x+86us
#define RF_CH           0x05	//RF通道,bit0~6工作通道频率
#define RF_SETUP        0x06	//RF寄存器，bit3:传输速率( 0:1M 1:2M);bit1~2:发射功率;bit0:噪声放大器增益
#define STATUS          0x07	//状态寄存器;bit0:TX FIFO满标志;bit1~3:接收数据通道号(最大:6);bit4:达到最多次重发次数
								//bit5:数据发送完成中断;bit6:接收数据中断
#define MAX_TX  		0x10	//达到最大发送次数中断
#define TX_OK   		0x20	//TX发送完成中断
#define RX_OK   		0x40	//接收到数据中断

#define OBSERVE_TX      0x08	//发送检测寄存器,bit7~4:数据包丢失计数器;bit3~0:重发计数器
#define CD              0x09	//载波检测寄存器,bit0:载波检测
#define RX_ADDR_P0      0x0A	//数据通道0接收地址，最大长度5个字节，低字节在前
#define RX_ADDR_P1      0x0B	//数据通道1接收地址，最大长度5个字节，低字节在前
#define RX_ADDR_P2      0x0C	//数据通道2接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define RX_ADDR_P3      0x0D	//数据通道3接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define RX_ADDR_P4      0x0E	//数据通道4接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define RX_ADDR_P5      0x0F	//数据通道5接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define TX_ADDR         0x10	//发送地址(低字节在前),ShockBurstTM模式下，RX_ADDR_P0与地址相等
#define RX_PW_P0        0x11	//接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1        0x12	//接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2        0x13	//接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3        0x14	//接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4        0x15	//接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5        0x16	//接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define NRF_FIFO_STATUS 0x17	//FIFO状态寄存器;bit0:RX FIFO寄存器空标志;bit1:RX FIFO满标志;bit2~3保留
								//bit4:TX FIFO 空标志;bit5:TX FIFO满标志;bit6:1,循环发送上一数据包.0,不循环								
#define DYNPD			0x1C
#define FEATRUE			0x1D
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Register Bits
#define MASK_RX_DR   	6 
#define MASK_TX_DS   	5 
#define MASK_MAX_RT  	4 
#define EN_CRC       	3 
#define CRCO         	2 
#define PWR_UP       	1 
#define PRIM_RX      	0 
#define ENAA_P5      	5 
#define ENAA_P4      	4 
#define ENAA_P3      	3 
#define ENAA_P2      	2 
#define ENAA_P1      	1 
#define ENAA_P0      	0 
#define ERX_P5       	5 
#define ERX_P4       	4 
#define ERX_P3       	3 
#define ERX_P2      	2 
#define ERX_P1       	1 
#define ERX_P0       	0 
#define AW_RERSERVED 	0x0 
#define AW_3BYTES    	0x1
#define AW_4BYTES    	0x2
#define AW_5BYTES    	0x3
#define ARD_250US    	(0x00<<4)
#define ARD_500US    	(0x01<<4)
#define ARD_750US    	(0x02<<4)
#define ARD_1000US   	(0x03<<4)
#define ARD_2000US   	(0x07<<4)
#define ARD_4000US   	(0x0F<<4)
#define ARC_DISABLE   	0x00
#define ARC_15        	0x0F
#define CONT_WAVE     	7 
#define RF_DR_LOW     	5 
#define PLL_LOCK      	4 
#define RF_DR_HIGH    	3 
#define PWR_18DB  		(0x00<<1)
#define PWR_12DB  		(0x01<<1)
#define PWR_6DB   		(0x02<<1)
#define PWR_0DB   		(0x03<<1)
#define RX_DR         	6 
#define TX_DS         	5 
#define MAX_RT        	4 
#define TX_FULL_0     	0 
#define RPD           	0 
#define TX_REUSE      	6 
#define TX_FULL_1     	5 
#define TX_EMPTY      	4 
#define RX_FULL       	1 
#define RX_EMPTY      	0 
#define DPL_P5        	5 
#define DPL_P4        	4 
#define DPL_P3        	3 
#define DPL_P2        	2 
#define DPL_P1        	1 
#define DPL_P0        	0 
#define EN_DPL        	2 
#define EN_ACK_PAY    	1 
#define EN_DYN_ACK    	0 
#define IRQ_ALL  ((1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT))

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Hardware Functions
uint8_t si24r1_read_write_byte(uint8_t tx_data);
uint8_t si24r1_read_reg(uint8_t reg);
void si24r1_read_buf(uint8_t reg, uint8_t *buf, uint8_t len);
void si24r1_write_reg(uint8_t reg, uint8_t data);
void si24r1_write_buf(uint8_t reg, uint8_t *buf, uint8_t len);
// Basic Functions
void si24r1_flush_tx_fifo(void);
void si24r1_flush_rx_fifo(void);
void si24r1_reuse_tx_payload(void);
void si24r1_nop(void);
uint8_t si24r1_get_status(void);
uint8_t si24r1_clear_irq_flag(uint8_t IRQ_Source);
void si24r1_clear_irq_all(void);
uint8_t si24r1_read_fifo_width(void);
uint8_t si24r1_read_rx_payload(uint8_t *rx_buf);
void si24r1_write_tx_payload_ack(uint8_t *tx_buf, uint8_t len);
uint8_t si24r1_write_tx_payload_nack(uint8_t *tx_buf, uint8_t len);
void si24r1_write_rx_payload_ack(uint8_t *tx_buf, uint8_t len);
void si24r1_set_tx_addr(uint8_t *tx_addr, uint8_t len);
void si24r1_set_rx_addr(uint8_t channel, uint8_t *rx_addr, uint8_t len);
void si24r1_set_speed(si24r1_speed_type speed);
void si24r1_enable_cont_wave(void);
void si24r1_disable_cont_wave(void);
void si24r1_set_power(uint8_t power_level);
void si24r1_set_channel(uint8_t freq);
void si24r1_set_mode(si24r1_mode_t mode);
// Extended Functions
bool si24r1_check_connection(void);
bool si24r1_init(si24r1_mode_t init_mode);
uint8_t si24r1_check_rx(uint8_t *rx_buf);
void si24r1_fast_tx(uint8_t *tx_buffer);
