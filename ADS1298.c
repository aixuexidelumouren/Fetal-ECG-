#include "ADS1298.h"
#include "nrf_drv_spi.h"

#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"

#include "nrf_queue.h"

#define SPI_INSTANCE 0                                               /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE); /**< SPI instance. */
static volatile bool spi_xfer_done;                                  /**< Flag used to indicate that SPI instance completed the transfer. */

#define ADS1298_SS_LOW		nrf_drv_gpiote_clr_task_trigger(ADS1298_SS_PIN)
#define ADS1298_SS_HIGH 	nrf_drv_gpiote_set_task_trigger(ADS1298_SS_PIN)

uint8_t recvbuf[27] = {0}; // to initail as all zeros to make sure no errors in followed
long a1;
long a2;
long a3;
long a4;
long channel_1;
long channel_2;
long channel_3;
long channel_RLD;

nrf_ppi_channel_t spi_start_transfer_channel;
nrf_ppi_channel_t spi_end_transfer_channel;

#define QUEUE_SIZE 600 //up to 600 bytes for each queue
NRF_QUEUE_DEF(uint8_t, m_head_sn_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(uint8_t, m_stat_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
//NRF_QUEUE_DEF(uint8_t, m_ecg1_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
//NRF_QUEUE_DEF(uint8_t, m_ecg2_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
//NRF_QUEUE_DEF(uint8_t, m_ecg3_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
//NRF_QUEUE_DEF(uint8_t, m_ecg4_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(uint8_t, m_ecg5_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(uint8_t, m_ecg6_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(uint8_t, m_ecg7_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(uint8_t, m_ecg8_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);

NRF_QUEUE_DEF(uint8_t, reserve_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);//6???
NRF_QUEUE_DEF(uint8_t, ecg_stat_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);//1???
NRF_QUEUE_DEF(uint8_t, battery_level_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);//3???
NRF_QUEUE_DEF(uint8_t, mac_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);//6???

uint8_t reserve_data[6]= {0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t ecg_stat = 0xFF;
uint8_t battery_level[3]= {0x00,0x00,0x00};
uint8_t ble_mac_data[6]= {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};//
static void spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context)
{
    static uint8_t count = 0;
    uint8_t i = 0;
    spi_xfer_done = true;
    NRF_SPIM0->RXD.PTR = (uint32_t)&recvbuf;
    static uint8_t head = 0xAA;  // just a protocol number 
    static uint32_t sn_count = 0; // sense number
    count++;
    if(count%4 ==1)
    {
        nrf_queue_push(&m_head_sn_queue, &head);
        nrf_queue_push(&m_head_sn_queue, &sn_count);
    }
		nrf_queue_push(&m_ecg5_queue, recvbuf + 15);
    nrf_queue_push(&m_ecg5_queue, recvbuf + 16);
    nrf_queue_push(&m_ecg5_queue, recvbuf + 17);
    
    nrf_queue_push(&m_ecg6_queue, recvbuf + 18);
    nrf_queue_push(&m_ecg6_queue, recvbuf + 19);
    nrf_queue_push(&m_ecg6_queue, recvbuf + 20);
    
    nrf_queue_push(&m_ecg7_queue, recvbuf + 21);
    nrf_queue_push(&m_ecg7_queue, recvbuf + 22);
    nrf_queue_push(&m_ecg7_queue, recvbuf + 23);
			
    nrf_queue_push(&m_ecg8_queue, recvbuf + 24);
    nrf_queue_push(&m_ecg8_queue, recvbuf + 25);
    nrf_queue_push(&m_ecg8_queue, recvbuf + 26);
//    
//    if(recvbuf[15]&0x80)
//    {
//        a1=(recvbuf[15]<<16)+(recvbuf[16]<<8)+recvbuf[17]-16777216;
//    }
//    else
//        a1=(recvbuf[15]<<16)+(recvbuf[16]<<8)+recvbuf[17];
//		

//    if(recvbuf[18]&0x80)
//    {
//        a2=(recvbuf[18]<<16)+(recvbuf[19]<<8)+recvbuf[20]-16777216;
//    }
//    else
//        a2=(recvbuf[18]<<16)+(recvbuf[19]<<8)+recvbuf[20];

//    if(recvbuf[21]&0x80)
//    {
//        a3=(recvbuf[21]<<16)+(recvbuf[22]<<8)+recvbuf[23]-16777216;
//    }
//    else
//        a3=(recvbuf[21]<<16)+(recvbuf[22]<<8)+recvbuf[23];

//    if(recvbuf[24]&0x80)
//    {
//        a4=(recvbuf[24]<<16)+(recvbuf[25]<<8)+recvbuf[26]-16777216;
//    }
//    else
//        a4=(recvbuf[24]<<16)+(recvbuf[25]<<8)+recvbuf[26];
//    

//    channel_RLD = - (a4+a2+a3)/3;
//    channel_1=a2-a3;
//    channel_2=a2-a4;
//    channel_3=a3-a4;
//    recvbuf[15]=channel_RLD>>16;
//    recvbuf[16]=(channel_RLD&0x00FF00)>>8;
//    recvbuf[17]=channel_RLD&0x0000FF;
//    
//    recvbuf[18]=channel_1>>16;
//    recvbuf[19]=(channel_1&0x00FF00)>>8;
//    recvbuf[20]=channel_1&0x0000FF;
//    
//    recvbuf[21]=channel_2>>16;
//    recvbuf[22]=(channel_2&0x00FF00)>>8;
//    recvbuf[23]=channel_2&0x0000FF;
//    
//    recvbuf[24]=channel_3>>16;
//    recvbuf[25]=(channel_3&0x00FF00)>>8;
//    recvbuf[26]=channel_3&0x0000FF;

    
//    nrf_queue_push(&m_ecg5_queue, recvbuf + 17);
//    nrf_queue_push(&m_ecg5_queue, recvbuf + 16);
//    nrf_queue_push(&m_ecg5_queue, recvbuf + 15);
//    
//    nrf_queue_push(&m_ecg6_queue, recvbuf + 20);
//    nrf_queue_push(&m_ecg6_queue, recvbuf + 19);
//    nrf_queue_push(&m_ecg6_queue, recvbuf + 18);
//    
//    nrf_queue_push(&m_ecg7_queue, recvbuf + 23);
//    nrf_queue_push(&m_ecg7_queue, recvbuf + 22);
//    nrf_queue_push(&m_ecg7_queue, recvbuf + 21);
//			
//    nrf_queue_push(&m_ecg8_queue, recvbuf + 26);
//    nrf_queue_push(&m_ecg8_queue, recvbuf + 25);
//    nrf_queue_push(&m_ecg8_queue, recvbuf + 24);

    if(count%4 ==0 && count >=4)
    {
        reserve_data[5] = sn_count;
        reserve_data[4] = sn_count>>8;
        reserve_data[3] = sn_count>>16;
        reserve_data[2] = sn_count>>24;
			  reserve_data[1] = count;
			  reserve_data[0] = count;
        for(i = 0; i < 6; i++)
        {
            nrf_queue_push(&reserve_queue, reserve_data + i);
        }
        nrf_queue_push(&ecg_stat_queue, &ecg_stat);
        for(i = 0; i < 3; i++)
        {
            nrf_queue_push(&battery_level_queue, battery_level + i);
        }
        for(i = 0; i < 6; i++)
        {
            nrf_queue_push(&mac_queue, ble_mac_data + i);
        }

        sn_count++;
    }

}

bool get_data_eight_chn(uint8_t *data)
{
    ret_code_t ret;
    uint8_t i = 0;
    uint8_t val = 0;
    ret = nrf_queue_pop(&m_head_sn_queue, &val);
    if (ret == NRF_SUCCESS)
    {
        *data = val;
        nrf_queue_pop(&m_head_sn_queue, data + 1);
        nrf_queue_pop(&m_ecg5_queue, data + 1 + 1);
        nrf_queue_pop(&m_ecg5_queue, data + 1 + 2);
        nrf_queue_pop(&m_ecg5_queue, data + 1 + 3);
        nrf_queue_pop(&m_ecg6_queue, data + 1 + 4);
        nrf_queue_pop(&m_ecg6_queue, data + 1 + 5);
        nrf_queue_pop(&m_ecg6_queue, data + 1 + 6);
        nrf_queue_pop(&m_ecg7_queue, data + 1 + 7);
        nrf_queue_pop(&m_ecg7_queue, data + 1 + 8);
        nrf_queue_pop(&m_ecg7_queue, data + 1 + 9);
        nrf_queue_pop(&m_ecg8_queue, data + 1 + 10);
        nrf_queue_pop(&m_ecg8_queue, data + 1 + 11);
        nrf_queue_pop(&m_ecg8_queue, data + 1 + 12);

        nrf_queue_pop(&m_ecg5_queue, data + 1 + 12 + 1);
        nrf_queue_pop(&m_ecg5_queue, data + 1 + 12 + 2);
        nrf_queue_pop(&m_ecg5_queue, data + 1 + 12 + 3);
        nrf_queue_pop(&m_ecg6_queue, data + 1 + 12 + 4);
        nrf_queue_pop(&m_ecg6_queue, data + 1 + 12 + 5);
        nrf_queue_pop(&m_ecg6_queue, data + 1 + 12 + 6);
        nrf_queue_pop(&m_ecg7_queue, data + 1 + 12 + 7);
        nrf_queue_pop(&m_ecg7_queue, data + 1 + 12 + 8);
        nrf_queue_pop(&m_ecg7_queue, data + 1 + 12 + 9);
        nrf_queue_pop(&m_ecg8_queue, data + 1 + 12 + 10);
        nrf_queue_pop(&m_ecg8_queue, data + 1 + 12 + 11);
        nrf_queue_pop(&m_ecg8_queue, data + 1 + 12 + 12);

        nrf_queue_pop(&m_ecg5_queue, data + 1 + 24 + 1);
        nrf_queue_pop(&m_ecg5_queue, data + 1 + 24 + 2);
        nrf_queue_pop(&m_ecg5_queue, data + 1 + 24 + 3);
        nrf_queue_pop(&m_ecg6_queue, data + 1 + 24 + 4);
        nrf_queue_pop(&m_ecg6_queue, data + 1 + 24 + 5);
        nrf_queue_pop(&m_ecg6_queue, data + 1 + 24 + 6);
        nrf_queue_pop(&m_ecg7_queue, data + 1 + 24 + 7);
        nrf_queue_pop(&m_ecg7_queue, data + 1 + 24 + 8);
        nrf_queue_pop(&m_ecg7_queue, data + 1 + 24 + 9);
        nrf_queue_pop(&m_ecg8_queue, data + 1 + 24 + 10);
        nrf_queue_pop(&m_ecg8_queue, data + 1 + 24 + 11);
        nrf_queue_pop(&m_ecg8_queue, data + 1 + 24 + 12);

        nrf_queue_pop(&m_ecg5_queue, data + 1 + 36 + 1);
        nrf_queue_pop(&m_ecg5_queue, data + 1 + 36 + 2);
        nrf_queue_pop(&m_ecg5_queue, data + 1 + 36 + 3);
        nrf_queue_pop(&m_ecg6_queue, data + 1 + 36 + 4);
        nrf_queue_pop(&m_ecg6_queue, data + 1 + 36 + 5);
        nrf_queue_pop(&m_ecg6_queue, data + 1 + 36 + 6);
        nrf_queue_pop(&m_ecg7_queue, data + 1 + 36 + 7);
        nrf_queue_pop(&m_ecg7_queue, data + 1 + 36 + 8);
        nrf_queue_pop(&m_ecg7_queue, data + 1 + 36 + 9);
        nrf_queue_pop(&m_ecg8_queue, data + 1 + 36 + 10);
        nrf_queue_pop(&m_ecg8_queue, data + 1 + 36 + 11);
        nrf_queue_pop(&m_ecg8_queue, data + 1 + 36 + 12);
        for(i = 0; i < 6; i++)
        {
            nrf_queue_pop(&reserve_queue, data + 1 + 49 + i);
        }
        nrf_queue_pop(&ecg_stat_queue, data + 1 + 55); //modified by luzy from push to pop
        for(i = 0; i < 3; i++)
        {
            nrf_queue_pop(&battery_level_queue, data + 1 + 56 + i);
        }
        for(i = 0; i < 6; i++)
        {
            nrf_queue_pop(&mac_queue, data + 1 + 59 + i);
        }
        return true;

    } else
    {
        return false;
    }

}

static void drdy_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

}

void ads1298_spi_init(void)
{
    ret_code_t ret;

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    spi_config.irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY;
    spi_config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.miso_pin = ADS1298_MISO_PIN;
    spi_config.mosi_pin = ADS1298_MOSI_PIN;
    spi_config.sck_pin = ADS1298_SCK_PIN;
    spi_config.mode = NRF_DRV_SPI_MODE_1;
    spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
    spi_config.orc = 0x00;
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

    ret = nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL);
    APP_ERROR_CHECK(ret);

    ret = nrf_drv_gpiote_init();

    nrf_drv_gpiote_out_config_t ss_pin_config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(true);
    ret = nrf_drv_gpiote_out_init(ADS1298_SS_PIN, &ss_pin_config);
    APP_ERROR_CHECK(ret);
    nrf_drv_gpiote_out_task_enable(ADS1298_SS_PIN);

    nrf_drv_gpiote_in_config_t drdy_pin_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    drdy_pin_config.pull = NRF_GPIO_PIN_PULLUP;
    ret = nrf_drv_gpiote_in_init(ADS1298_DRDY_PIN, &drdy_pin_config, drdy_pin_handler);
    APP_ERROR_CHECK(ret);
    nrf_drv_gpiote_in_event_enable(ADS1298_DRDY_PIN, true);

    ret = nrf_drv_ppi_init();
    APP_ERROR_CHECK(ret);
    ret = nrf_drv_ppi_channel_alloc(&spi_start_transfer_channel);
    APP_ERROR_CHECK(ret);
    ret = nrf_drv_ppi_channel_assign(spi_start_transfer_channel, nrf_drv_gpiote_in_event_addr_get(ADS1298_DRDY_PIN), nrf_drv_gpiote_clr_task_addr_get(ADS1298_SS_PIN));
    APP_ERROR_CHECK(ret);
    ret = nrf_drv_ppi_channel_fork_assign(spi_start_transfer_channel, nrf_drv_spi_start_task_get(&spi));
    APP_ERROR_CHECK(ret);

    ret = nrf_drv_ppi_channel_alloc(&spi_end_transfer_channel);
    APP_ERROR_CHECK(ret);
    ret = nrf_drv_ppi_channel_assign(spi_end_transfer_channel, nrf_drv_spi_end_event_get(&spi), nrf_drv_gpiote_set_task_addr_get(ADS1298_SS_PIN));
    APP_ERROR_CHECK(ret);
}

void ads1298_ppi_recv_start(void) {

    ret_code_t ret;

    NRF_SPIM0->TXD.MAXCNT = 0;
    NRF_SPIM0->RXD.MAXCNT = 27;
    NRF_SPIM0->TXD.LIST =	0;
    //NRF_SPIM0->TXD.PTR = NULL;
    NRF_SPIM0->RXD.LIST =	1;
    NRF_SPIM0->RXD.PTR = (uint32_t)&recvbuf;

    ret = nrf_drv_ppi_channel_enable(spi_start_transfer_channel);
    APP_ERROR_CHECK(ret);
    ret = nrf_drv_ppi_channel_enable(spi_end_transfer_channel);
    APP_ERROR_CHECK(ret);

}

/* 
spi sending function for pins communication
*/
static void ads1298_spi_send(uint8_t send)
{
    spi_xfer_done = false; // sign for transfer complete
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &send, 1, NULL, 0)); // spi transfer function, send 1 bit and receieve 0 bit (don't receive)

    nrf_delay_us(2); // delay for 2 mircosecond for processing

    while (!spi_xfer_done)  // start to check if the transfer is finished
        __WFE();  // wait for event, a low energy waiting function for wait the completion for spi transferring
}

static uint8_t ads1298_spi_recv(void)
{
    uint8_t ret = 0;

    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, &ret, 1));

    nrf_delay_us(2);

    while (!spi_xfer_done)
        __WFE();

    return ret;
}

uint8_t ads1298_read_register(uint8_t reg)
{
    uint8_t val = 0;
    ads1298_read_multiple_register(reg, &val, 1);
    return val;
}

bool ads1298_read_multiple_register(uint8_t reg, uint8_t* val, uint8_t num)
{

    ADS1298_SS_LOW;

    ads1298_spi_send(ADS129X_CMD_RREG | (reg & 0x1F));
    ads1298_spi_send((num - 1) & 0x1F);

    for(int i = 0; i < num; i++)
        *(val + i) = ads1298_spi_recv();

    ADS1298_SS_HIGH;

    return true;

}

bool ads1298_write_register(uint8_t reg, uint8_t val)
{

    ads1298_write_multiple_register(reg, &val, 1);

    return true;

}

bool ads1298_write_multiple_register(uint8_t reg, uint8_t* val, uint8_t num)
{

    ADS1298_SS_LOW;

    ads1298_spi_send(ADS129X_CMD_WREG | (reg & 0x1F));
    ads1298_spi_send((num - 1) & 0x1F);

    for(int i = 0; i < num; i++)
        ads1298_spi_send(*(val + i));

    ADS1298_SS_HIGH;

    return true;

}

/*Power off the device and use spi transfer the micro(command) and then power on again*/
bool ads1298_write_command(uint8_t val)
{

    ADS1298_SS_LOW;

    ads1298_spi_send(val);

    ADS1298_SS_HIGH;

    return true;

}
