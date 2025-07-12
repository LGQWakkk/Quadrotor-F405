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
