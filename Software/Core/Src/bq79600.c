#include "bq79600.h"
#include "SEGGER_RTT.h"
#include "bq79616_def.h"
#include <string.h>
#include <main.h>
#define MAX_INSTANCE 1
static bq79600_t instance_list[MAX_INSTANCE] = {0};

void bq79600_construct_command(bq79600_t *instance, REQ_TYPE req_type, uint8_t addr, uint16_t reg_addr,
                               uint8_t data_len, uint8_t *data) {
  uint8_t *tx_buf = instance->tx_buf;
  *tx_buf++ = 0x80 | (req_type << 4) | ((req_type & 1) ? ((data_len - 1) & 0x0F) : 0);
  if (req_type < 2) *tx_buf++ = addr & 0x3F;
  *tx_buf++ = (reg_addr >> 8) & 0xFF;
  *tx_buf++ = reg_addr & 0xFF;
  if (data)
    for (int i = 0; i < data_len; i++) *tx_buf++ = data[i];
  else
    *tx_buf++ = data_len - 1;
  if (req_type >= 2) data_len = 0;
  uint16_t crc = bq79600_bsp_crc(instance->tx_buf, 4 + data_len);
  *tx_buf++ = crc & 0xFF;
  *tx_buf++ = (crc >> 8) & 0xFF;
  instance->tx_len = tx_buf - instance->tx_buf;
}

void bq79600_tx(bq79600_t *instance) {
  instance->ready = 0;
  SEGGER_RTT_printf(0, "[BQ79600] TX: ");
  for (int i = 0; i < instance->tx_len; i++) SEGGER_RTT_printf(0, "%02X ", instance->tx_buf[i]);
  SEGGER_RTT_printf(0, "\n");
  switch (instance->mode) {
    case BQ_UART:
      bq79600_bsp_uart_tx(instance);
      break;
    default:
      break;
  }
}

void bq79600_rx_callback(bq79600_t *instance) {
  if (instance->rx_len < 6) return;
  SEGGER_RTT_printf(0, "[BQ79600] RX[%d]: ", instance->rx_len);
  for (int i = 0; i < instance->rx_len; i++) SEGGER_RTT_printf(0, "%02X ", instance->rx_buf[i]);
  SEGGER_RTT_printf(0, "\n");

  size_t idx = 0;
  uint8_t crc_buf[128 + 6];
  while (idx < instance->rx_len) {
    for (int i = 0; i < 4; i++) crc_buf[i] = instance->rx_buf[idx++];
    uint8_t len = (crc_buf[0] & 0x7F) + 1;
    for (int i = 0; i < len; i++) crc_buf[4 + i] = instance->rx_buf[idx++];
    crc_buf[4 + len] = instance->rx_buf[idx++];
    crc_buf[5 + len] = instance->rx_buf[idx++];
    uint16_t crc = bq79600_bsp_crc(crc_buf, len + 4);
    uint16_t crc_rx = (crc_buf[4 + len] << 8) | crc_buf[5 + len];
    if (!(crc ^ crc_rx)) {
      SEGGER_RTT_printf(0, "[BQ79600] CRC error: %04X %04X\n", crc, crc_rx);
      instance->fault = 1;
      return;
    }
  }
  instance->fault = 0;
  instance->ready = 1;
}

void bq79600_read_reg(bq79600_t *instance, uint8_t dev_addr, uint16_t reg_addr, uint8_t *data) {
  bq79600_construct_command(instance, SINGLE_DEVICE_READ, dev_addr, reg_addr, 1, NULL);
  bq79600_tx(instance);
  bq79600_bsp_ready(instance);
  *data = instance->rx_buf[4];
}
extern UART_HandleTypeDef huart4;
void bq79600_write_reg(bq79600_t *instance, uint8_t dev_addr, uint16_t reg_addr, uint8_t *data,
                       uint8_t data_len) {
  bq79600_construct_command(instance, SINGLE_DEVICE_WRITE, dev_addr, reg_addr, data_len, data);
  //bq79600_tx(instance);
  HAL_UART_Transmit(&huart4, instance->tx_buf , instance->tx_len,100);
}

bq79600_t *open_bq79600_instance(uint32_t id) {
  if (id >= MAX_INSTANCE) return NULL;
  return &instance_list[id];
}

void bq79600_wakeup(bq79600_t *instance) {
  bq79600_bsp_wakeup(instance);
  switch (instance->mode) {
    case BQ_UART:
      bq79600_bsp_uart_init(instance);
      break;
    default:
      break;
  }
  instance->state = BQ_ACTIVATE;
  SEGGER_RTT_printf(0, "[BQ79600] wakeup.\n");
}

bq79600_error_t bq79600_auto_addressing(bq79600_t *instance, const size_t n_devices) {
  uint8_t buf = 0;
  for (int addr = 0x343; addr < 0x34B; addr++) {
    bq79600_construct_command(instance, STACK_WRITE, 0, addr, 1, &buf);
    bq79600_tx(instance);
  }
  instance->fault = 0 ;
  // Enable auto addressing
  buf = 0x01;
  bq79600_construct_command(instance, BROADCAST_WRITE, 0, CONTROL1, 1, &buf);
  bq79600_tx(instance);
  // brdcast write consecutively to 0x306
  for (size_t i = 0; i < n_devices; i++) {
    buf = i;
    bq79600_construct_command(instance, BROADCAST_WRITE, 0, DIR0_ADDR, 1, &buf);
    bq79600_tx(instance);
  }
  // brdcast write 0x02 to address 0x308 (set BQ7961X-Q1 as stack device )
  buf = 0x02;
  bq79600_construct_command(instance, BROADCAST_WRITE, 0, COMM_CTRL, 1, &buf);
  bq79600_tx(instance);

  buf = 0x03;
  bq79600_construct_command(instance, SINGLE_DEVICE_WRITE, n_devices - 1, COMM_CTRL, 1, &buf);
  bq79600_tx(instance);

  for (int addr = 0x343; addr < 0x34B; addr++) {
    bq79600_construct_command(instance, STACK_READ, 0, addr, 1, NULL);
    bq79600_tx(instance);
    bq79600_bsp_ready(instance);
   // uint8_t comm_ctrl[6] = {0};
    //if (HAL_UART_Receive(&huart4, instance->rx_buf , instance->rx_len ,100) != HAL_OK)  return BQ_ERROR  ;
   // HAL_UART_Receive(&huart4, comm_ctrl , 6 ,100);
   if (instance->fault)
	   return BQ_ERROR;




  }

  for (size_t i = 0; i < n_devices; i++) {
    bq79600_construct_command(instance, SINGLE_DEVICE_READ, i, DIR0_ADDR, 1, NULL);
    bq79600_tx(instance);
   // if (HAL_UART_Receive(&huart4, instance->rx_buf , instance->rx_len ,100) != HAL_OK)  return BQ_ERROR  ;
     bq79600_bsp_ready(instance);
    if (instance->fault)
    	return BQ_ERROR;
  }
  return BQ_SUCCESS;
}

void initalize_communication(bq79600_t *instance,UART_HandleTypeDef *uart_port , int n_devices, int n_cells_per_device)
{



		      uint8_t buf = 0x20;
		      bq79600_write_reg(instance, 0x00, CONTROL1, &buf, 1);
		      HAL_Delay(12 * n_devices);

		      bq79600_error_t err = bq79600_auto_addressing(instance, n_devices);
		      if (err) {
		    	//  Message autoadress = {0};
		    	//  strcpy(autoadress.Buf, "Autoadressing failed!\n0");
		    	//  autoadress.Timestamp = HAL_GetTick();
		    	 // osMessageQueuePut(Messages_QueueHandle, &autoadress, 0, 50);
		    	  }
		      else
		      {
		    	 // Message autoadress = {0};
		    	 // strcpy(autoadress.Buf, "Autoadressing succesful!\n0");
		    	 // autoadress.Timestamp = HAL_GetTick();
		    	 // osMessageQueuePut(Messages_QueueHandle, &autoadress, 0, 50);
		      }


		      /* Set long communication timeout */
		      buf = 0x0A;  // CTL_ACT=1 | CTL_TIME=010 (2s)
		      bq79600_construct_command(instance, STACK_WRITE, 0, COMM_TIMEOUT_CONF, 5, &buf);
		      bq79600_tx(instance);
		      HAL_Delay(1);



		      buf = 0x01; // 0x01
		      	      bq79600_construct_command(instance, STACK_WRITE, 0, CONTROL2, 1, &buf); // enable T_REF adc reading
		      	      bq79600_tx(instance);
		      	    HAL_Delay(10);






		      /* Config stack device ADCs */
		      buf = n_cells_per_device - 6;
		      bq79600_construct_command(instance, STACK_WRITE, 0, ACTIVE_CELL, 1, &buf);
		      bq79600_tx(instance);

		  	  buf =0x9; // 0x9; // 0x09;
		      bq79600_construct_command(instance, STACK_WRITE, 0, GPIO_CONF1, 1, &buf);
		      bq79600_tx(instance);
		      buf =0x9; // 0x9; // 0x09;
		      	      bq79600_construct_command(instance, STACK_WRITE, 0, GPIO_CONF2, 1, &buf);
		      	      bq79600_tx(instance);
		      	    buf =0x9; // 0x9; // 0x09;
		      	    	      bq79600_construct_command(instance, STACK_WRITE, 0, GPIO_CONF3, 1, &buf);
		      	    	      bq79600_tx(instance);
		      	    	    buf =0x9; // 0x9; // 0x09;
		      	    	    	      bq79600_construct_command(instance, STACK_WRITE, 0, GPIO_CONF4, 1, &buf);
		      	    	    	      bq79600_tx(instance);

		      buf = 0x06;
		      bq79600_construct_command(instance, STACK_WRITE, 0, ADC_CTRL1, 1, &buf);
		      bq79600_tx(instance);
		      HAL_Delay(1 * n_devices);

		      buf = 0x06;
		      bq79600_construct_command(instance, STACK_WRITE, 0, ADC_CTRL1, 1, &buf);
		      bq79600_tx(instance);
		      HAL_Delay(1 * n_devices);




	/*
		      buf = 0x01; // 0x01
		      bq79600_construct_command(bms_instance, STACK_WRITE, 0, CONTROL2, 1, &buf); // enable T_REF adc reading
		      bq79600_tx(bms_instance);
		      osDelay(1 * n_devices); */




	  /*
	    	    buf = 0x6;  //
	    	    bq79600_construct_command(bms_instance, STACK_WRITE, 0, ADC_CTRL3, 1, &buf);
	    	    bq79600_tx(bms_instance);
	    	    osDelay(1 * n_devices);
	 */




		      /*  Setup OV, UV for balancing  */

		      uint8_t ov_threshold = 0x22;//0x22; // 4175 mV threshold value
		      bq79600_construct_command(instance, STACK_WRITE, 0, OV_THRESH, 1, &ov_threshold);
		      bq79600_tx(instance);
		      HAL_Delay(1 * n_devices);
		      uint8_t uv_threshold = 0x22; // 3000 mV threshold value
		      bq79600_construct_command(instance, STACK_WRITE, 0, UV_THRESH, 1, &uv_threshold);
		      bq79600_tx(instance);
		      HAL_Delay(1 * n_devices);

		      buf = 0x5 ; //0x5;
		      bq79600_construct_command(instance, STACK_WRITE, 0, OVUV_CTRL, 1, &buf); // Set mode to run OV and UV round robin on all cells
		      bq79600_tx(instance);														// and start OV UV comparators


		      uint8_t ot_threshold = 0x3; // 0x3 = 80C threshold
		      bq79600_construct_command(instance, STACK_WRITE, 0, OTUT_THRESH, 1, &ot_threshold);
		      bq79600_tx(instance);

		      buf = 0x5 ; //0x5;
		      bq79600_construct_command(instance, STACK_WRITE, 0, OTUT_CTRL, 1, &buf); // Set mode to run OT and UT round robin on all cells
		      bq79600_tx(instance);
}
