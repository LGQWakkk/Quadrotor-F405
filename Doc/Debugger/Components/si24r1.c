// 20250702 Wakkk
#include "si24r1.h"
#include "system.h"

uint8_t si24r1_tx_address[5] = {DEFAULT_TX_ADDR};
uint8_t si24r1_rx_address[5] = {DEFAULT_RX_ADDR};

// 仅仅全双工传输一个字节 不操作CS
uint8_t si24r1_read_write_byte(uint8_t tx_data)
{
	uint8_t rx_data = 0x00;
	HAL_SPI_TransmitReceive(&SI24R1_SPI_HANDLE, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
	return rx_data;
}

// 读取寄存器: 首先发送一字节地址和读标志位 之后读取一字节
uint8_t si24r1_read_reg(uint8_t reg)
{
    SI24R1_CS_LOW();
    si24r1_read_write_byte(NRF_READ_REG|reg);
    uint8_t data = si24r1_read_write_byte(0xFF);
    SI24R1_CS_HIGH();
    return data;
}

// 读取寄存器多个字节: 首先发送一字节地址和读标志位 之后读取指定长度字节
void si24r1_read_buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    SI24R1_CS_LOW();
    si24r1_read_write_byte(NRF_READ_REG|reg);
    HAL_SPI_Receive(&SI24R1_SPI_HANDLE, buf, len, HAL_MAX_DELAY);
    SI24R1_CS_HIGH();
}

// 写入寄存器: 首先发送一字节地址和写标志位 之后写入一字节
void si24r1_write_reg(uint8_t reg, uint8_t data)
{
    SI24R1_CS_LOW();
    si24r1_read_write_byte(NRF_WRITE_REG|reg);
    si24r1_read_write_byte(data);
    SI24R1_CS_HIGH();
}

// 写入寄存器多个字节: 首先发送一字节地址和写标志位 之后写入指定长度字节
void si24r1_write_buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    SI24R1_CS_LOW();
    si24r1_read_write_byte(NRF_WRITE_REG|reg);
    HAL_SPI_Transmit(&SI24R1_SPI_HANDLE, buf, len, HAL_MAX_DELAY);
    SI24R1_CS_HIGH();
}

void si24r1_flush_tx_fifo(void)
{
    SI24R1_CS_LOW();
    si24r1_read_write_byte(FLUSH_TX);
    SI24R1_CS_HIGH();
}

void si24r1_flush_rx_fifo(void)
{
    SI24R1_CS_LOW();
    si24r1_read_write_byte(FLUSH_RX);
    SI24R1_CS_HIGH();
}

void si24r1_reuse_tx_payload(void)
{
    SI24R1_CS_LOW();
    si24r1_read_write_byte(REUSE_TX_PL);
    SI24R1_CS_HIGH();
}

void si24r1_nop(void)
{
    SI24R1_CS_LOW();
    si24r1_read_write_byte(NOP);
    SI24R1_CS_HIGH();
}

uint8_t si24r1_get_status(void)
{
    SI24R1_CS_LOW();
    uint8_t status = si24r1_read_write_byte(NRF_READ_REG+STATUS);
    SI24R1_CS_HIGH();
    return status;
}

uint8_t si24r1_clear_irq_flag(uint8_t IRQ_Source)
{
    IRQ_Source &= (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT);
    uint8_t status = si24r1_get_status();
    SI24R1_CS_LOW();
    si24r1_read_write_byte(NRF_WRITE_REG+STATUS);
    si24r1_read_write_byte(IRQ_Source|status);
    SI24R1_CS_HIGH();
    return (si24r1_get_status());
}

void si24r1_clear_irq_all(void)
{
    uint8_t status = si24r1_get_status();
    SI24R1_CS_LOW();
    si24r1_read_write_byte(NRF_WRITE_REG+STATUS);
    si24r1_read_write_byte(status);
    SI24R1_CS_HIGH();
}

uint8_t si24r1_read_fifo_width(void)
{
    SI24R1_CS_LOW();
    si24r1_read_write_byte(R_RX_PL_WID);
    uint8_t data = si24r1_read_write_byte(0xFF);
    SI24R1_CS_HIGH();
    return data;
}

uint8_t si24r1_read_rx_payload(uint8_t *rx_buf)
{
    uint8_t data_len, channel;
    channel = (si24r1_read_reg(STATUS)>>1)&0x07;
    data_len = si24r1_read_fifo_width();
    SI24R1_CS_LOW();
    si24r1_read_write_byte(RD_RX_PLOAD);
    for(channel = 0; channel<data_len; channel ++){
        *(rx_buf+channel) = si24r1_read_write_byte(0xFF);
    }
    SI24R1_CS_HIGH();
    si24r1_flush_rx_fifo();
    return data_len;
}

void si24r1_write_tx_payload_ack(uint8_t *tx_buf, uint8_t len)
{
    uint8_t length = (len>32)?32:len;
    si24r1_flush_tx_fifo();
    SI24R1_CS_LOW();
    si24r1_read_write_byte(WR_TX_PLOAD);
    for(uint8_t i = 0; i < length; i++){
        si24r1_read_write_byte(*(tx_buf + i));
    }
    SI24R1_CS_HIGH();	
}

// 性能测试: SPI-SCK@16MHZ 发送长度len=32Bytes
// WR_TX_PLOAD_NACK+DATA 传输耗时: 365us
// 发送时间(到IRQ下降沿): 453us
// 检测并清除中断时间(IRQ下降沿到上升沿): 38us
// 此函数连续调用最大发送频率(IRQ频率): 周期865us(1156Hz)
uint8_t si24r1_write_tx_payload_nack(uint8_t *tx_buf, uint8_t len)
{
    if(len>32||len==0)return 0;
    SI24R1_CS_LOW();
    si24r1_read_write_byte(WR_TX_PLOAD_NACK);
    while(len--){
        si24r1_read_write_byte(*tx_buf);
        tx_buf++;
    }
    SI24R1_CS_HIGH();
    while(SI24R1_GET_IRQ_STATUS());
    uint8_t status = si24r1_read_reg(STATUS);
    si24r1_write_reg(STATUS, status);
    return status;
}

// 在接收模式下写入发送数据
void si24r1_write_rx_payload_ack(uint8_t *tx_buf, uint8_t len)
{
    len = (len>32)?32:len;
    SI24R1_CS_LOW();
    si24r1_read_write_byte(W_ACK_PLOAD);
    for(uint8_t i = 0; i < len; i++){
        si24r1_read_write_byte(*(tx_buf+i));
    }
    SI24R1_CS_HIGH();
}

void si24r1_set_tx_addr(uint8_t *tx_addr, uint8_t len)
{
    len = (len>5)?5:len;
    si24r1_write_buf(TX_ADDR, tx_addr, len);
}

// 设置接收通道地址
// channel: 通道 0-5 tx_addr: 地址存放地址 len: 地址长度 不大于5字节
void si24r1_set_rx_addr(uint8_t channel, uint8_t *rx_addr, uint8_t len)
{
    len = (len>5)?5:len;
    channel = (channel>5)?5:channel;
    si24r1_write_buf(RX_ADDR_P0+channel, rx_addr, len);
}

void si24r1_set_speed(si24r1_speed_type speed)
{
	uint8_t data = si24r1_read_reg(RF_SETUP);
	data &= ~((1<<5)|(1<<3));
	if(speed == SPEED_250K){
		data |= (1<<5);
	}
	else if(speed == SPEED_1M){
   	    data &= ~((1<<5)|(1<<3));
	}
	else if(speed == SPEED_2M){
		data |= (1<<3);
	}
	si24r1_write_reg(RF_SETUP, data);
}

void si24r1_enable_cont_wave(void)
{
    uint8_t data = si24r1_read_reg(RF_SETUP);
    data |= 0x80;
    si24r1_write_reg(RF_SETUP, data);
}

void si24r1_disable_cont_wave(void)
{
    uint8_t data = si24r1_read_reg(RF_SETUP);
    data &= (~0x80);
    si24r1_write_reg(RF_SETUP, data);
}

void si24r1_set_power(uint8_t power_level)
{
    if(power_level>7) power_level = 7;
    uint8_t data = si24r1_read_reg(RF_SETUP) & ~0x07;
    data |= power_level;  
    si24r1_write_reg(RF_SETUP, data);
}

void si24r1_set_channel(uint8_t freq)
{
    si24r1_write_reg(RF_CH, freq & 0x7F);
}

void si24r1_set_mode(si24r1_mode_t mode)
{
    uint8_t controlreg = 0;
    controlreg = si24r1_read_reg(CONFIG);
    if(mode == MODE_TX){
        controlreg &= ~(1<< PRIM_RX);
    }else{
        controlreg |= (1<< PRIM_RX); 
    }
    si24r1_write_reg(CONFIG, controlreg);
}

bool si24r1_check_connection(void)
{
    uint8_t test_buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
    uint8_t read_buf[5] = {0};
    si24r1_write_buf(TX_ADDR, test_buf, 5);
    si24r1_read_buf(TX_ADDR, read_buf, 5);
    bool ret = true;
    for(uint8_t i = 0; i < 5; i++){
        if(test_buf[i] != read_buf[i]){
            ret = false;
            break;
        }
    }
    return ret;
}

bool si24r1_init(si24r1_mode_t init_mode)
{
    bool check_ok = false;
    for(uint8_t i=0; i<SI24R1_CHECK_MAX; i++){
        if(si24r1_check_connection()){
            check_ok = true;
            break;
        }else{
            slog("si24r1 connection check failed. retry...\r\n");
        }
        HAL_Delay(10);
    }
    if(!check_ok)return false;
    slog("SI24R1 connection check OK.\r\n");
    SI24R1_CE_LOW();
    HAL_Delay(1);
    si24r1_clear_irq_all();
    si24r1_write_reg(DYNPD, (1 << 0));
    si24r1_write_reg(FEATRUE, 0x07);
    si24r1_write_reg(CONFIG, (1 << EN_CRC) | (1 << PWR_UP));
    si24r1_write_reg(EN_AA, (1 << ENAA_P0));
    si24r1_write_reg(EN_RXADDR, (1 << ERX_P0));
    si24r1_write_reg(SETUP_AW, AW_5BYTES);
    si24r1_write_reg(SETUP_RETR, ARD_4000US | (REPEAT_CNT & 0x0F));
    si24r1_write_reg(RF_SETUP, 0x26);
    si24r1_set_tx_addr(si24r1_tx_address, 5);
    si24r1_set_rx_addr(0, si24r1_rx_address, 5);
    si24r1_set_speed(SPEED_1M);
    si24r1_set_channel(60);
    si24r1_set_power(5);
    si24r1_set_mode(init_mode);
    SI24R1_CE_HIGH();
    HAL_Delay(1);
    return true;
}

// 检测SI24R1是否接收到数据
// 返回0: 没有接收到数据
// 其他值表示收到数据包的字节数量
uint8_t si24r1_check_rx(uint8_t *rx_buf)
{
    uint8_t rxlength = 0;
    uint8_t status = si24r1_read_reg(STATUS);
    si24r1_write_reg(STATUS, status); //清除中断
    if(status & RX_OK)	//接收到数据
    {
        rxlength = si24r1_read_reg(R_RX_PL_WID); // 读取接收长度
        si24r1_read_buf(RD_RX_PLOAD, rx_buf, rxlength); // 读取接收数据
        si24r1_flush_rx_fifo(); // 清空接收FIFO
        return rxlength; //返回接收到的数据字节数
    }
    return 0; //没有收到数据
}

// SI24R1以最高效率发送数据包 无应答
void si24r1_fast_tx(uint8_t *tx_buffer)
{
    SI24R1_CS_LOW();
    si24r1_read_write_byte(WR_TX_PLOAD_NACK);
    HAL_SPI_Transmit(&SI24R1_SPI_HANDLE, tx_buffer, 32, HAL_MAX_DELAY);
    SI24R1_CS_HIGH();
    // 等待发送完毕
    while(SI24R1_GET_IRQ_STATUS()); // 高电平 未产生中断
    // 清除中断
    uint8_t status = si24r1_read_reg(STATUS);
    si24r1_write_reg(STATUS, status);
}

// //非阻塞接收数据 同时处理ACK PAYLOAD数据包
// uint8_t si24r1_rx_packet_nowait(uint8_t* rxbuf)
// {
// 	uint8_t _status = 0, _rxlength = 0;
// 	_status = si24r1_read_reg(STATUS);
// 	si24r1_read_reg(STATUS, _status);//清除中断
// 	if(_status & RX_OK)	//接收到数据
// 	{
// 		_rxlength = si24r1_read_reg(R_RX_PL_WID);
// 		si24r1_read_buf(RD_RX_PLOAD,rxbuf,_rxlength);
// 		si24r1_read_reg(FLUSH_RX,0xff);
// 		return _rxlength; //返回接收到的数据字节数
// 	}
//   if((_status & TX_OK) && (enable_ack_payload)){ //作为接收端收到TXDS表示ACK PAYLOAD发送完成
//     // TX FIFO中ACK PAYLOAD已经自动清除
//     // 这里再次进行手动清除
//     SI24R1_CS_LOW();	
//     si24r1_read_write_byte(FLUSH_TX);
//     SI24R1_CS_HIGH();	
//     // 再次填充TX FIFO 准备下一次返回数据
//     si24r1_write_rx_payload_ack(si24r1_ack_payload_buffer, 32);
//     ack_payload_count ++;//返回的数据包计数
//   }
// 	return 0;//没有收到数据	
// }

// //发送数据包 阻塞等待发送完毕
// //txbuf: 待发送数据缓冲区
// //length: 待发送数据长度
// //timeout_ms: 最大等待时间
// //返回值:  MAX_TX：达到最大重发次数  TX_OK：发送完成  0xFF:其他原因
// uint8_t si24r1_tx_packet(uint8_t *txbuf, uint8_t length, uint16_t timeout_ms)
// {
// 	uint8_t _status = 0;
// 	uint16_t _1ms_count = 0;
// 	SI24R1_CS_LOW();
// 	si24r1_read_write_byte(FLUSH_TX);//清空之前要发送的数据
// 	SI24R1_CS_HIGH();

// 	SI24R1_CE_LOW();		
// 	si24r1_write_buf(WR_TX_PLOAD, txbuf, length);	//写数据到TX BUF 32字节  TX_PLOAD_WIDTH
// 	SI24R1_CE_HIGH();			                        //启动发送

// 	while(SI24R1_GET_IRQ_STATUS()){               //阻塞等待接收中断 每1ms检测一次
//     HAL_Delay(1);
//     _1ms_count ++;
//     if(_1ms_count >= timeout_ms)break;
// 	}
// 	if(_1ms_count >= timeout_ms){
//     log("si24r1: tx packet timeout!\r\n");
//   }else{
//     log("si24r1: tx packet done! wait time: %dms\r\n", _1ms_count);
//   }

// 	_status = si24r1_read_reg(STATUS);		//读状态寄存器
// 	si24r1_read_reg(STATUS, _status); //清除TX_DS或MAX_RT中断标志

// 	if(_status & MAX_TX){
// 		si24r1_read_reg(FLUSH_TX,0xff);	//清除TX FIFO寄存器
//     log("si24r1: tx failed: MAX TX\r\n");
// 		return MAX_TX; 
// 	}
// 	if(_status & TX_OK){
//     log("si24r1: tx success\r\n");
// 		return TX_OK;
// 	}
// 	log("si24r1: tx failed: other error\r\n");
// 	return 0xFF;	//其他原因发送失败
// }

// //阻塞等待接收数据
// //rxbuf: 接收数据存放地址
// //timeout_ms: 最大等待时间
// //ret: 接收到的数据字节数 若超时则返回0
// uint8_t si24r1_rx_packet_wait(uint8_t *rxbuf, uint16_t timeout_ms)
// {
//   // RF24L01_CLEAR_IRQ_STATUS();
// 	uint8_t _status = 0, _rxlength = 0;
//   uint16_t _1ms_count = 0;//超时计数器
// 	SI24R1_CS_LOW();	
// 	si24r1_read_write_byte(FLUSH_RX);//首先清空之前有可能收到的数据
// 	SI24R1_CS_HIGH();
// 	while(SI24R1_GET_IRQ_STATUS()){//阻塞等待接收中断 每1ms检测一次
//     HAL_Delay(1);
//     _1ms_count ++;
//     if(_1ms_count >= timeout_ms)break;
// 	}
// 	if(_1ms_count >= timeout_ms){
//     log("si24r1: rx packet timeout!\r\n");
//   }else{
//     log("si24r1: rx packet received! wait time: %dms\r\n", _1ms_count);
//   }
// 	_status = si24r1_read_reg(STATUS);		//读状态寄存器
// 	si24r1_read_reg(STATUS,_status);		//清中断标志
// 	if(_status&RX_OK)	                //接收到数据
// 	{
// 		_rxlength = si24r1_read_reg(R_RX_PL_WID);		  //读取接收到的数据个数
// 		si24r1_read_buf(RD_RX_PLOAD,rxbuf,_rxlength);	//接收到数据 
// 		si24r1_read_reg(FLUSH_RX,0xff);				        //清除RX FIFO
// 		return _rxlength; 
// 	}	
// 	return 0;	//没有收到数据	
// }
