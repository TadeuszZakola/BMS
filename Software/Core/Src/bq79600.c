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
  buf =0x1; //0x81; //0x1
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



		      uint8_t buf = 0x20;//0x20;// 0x20;//0x20;
		      bq79600_write_reg(instance, 0x00, CONTROL1, &buf, 1);
		      HAL_Delay(12 * n_devices);

		      bq79600_error_t err = bq79600_auto_addressing(instance, n_devices);
		      if (err) {
		    	//  int autoadress = 0 ;
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
		      buf = 0x010;  // CTL_ACT=1 | CTL_TIME=010 (2s)
		      bq79600_construct_command(instance, STACK_WRITE, 0, COMM_TIMEOUT_CONF, 1, &buf);
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

		      buf = 0x8; // set lowpass filter
		      bq79600_construct_command(instance, STACK_WRITE, 0, ADC_CONF1, 1, &buf);
		      bq79600_tx(instance);
		      HAL_Delay(1 * n_devices);

		      buf = 0x06;
		      bq79600_construct_command(instance, STACK_WRITE, 0, ADC_CTRL1, 1, &buf);
		      bq79600_tx(instance);
		      HAL_Delay(1 * n_devices);







		      /*  Setup OV, UV for balancing  */

		      uint8_t ov_threshold = 0x22;// 0X1E - 3800mv //0x22; // 4175 mV threshold value
		      bq79600_construct_command(instance, STACK_WRITE, 0, OV_THRESH, 1, &ov_threshold);
		      bq79600_tx(instance);
		      HAL_Delay(1 * n_devices);
		      uint8_t uv_threshold = 0x22; // 3000 mV threshold value
		      bq79600_construct_command(instance, STACK_WRITE, 0, UV_THRESH, 1, &uv_threshold);
		      bq79600_tx(instance);
		      HAL_Delay(1 * n_devices);
		      uint8_t uv_balance_threshold = 0x30 ; // 3.3V
		      bq79600_construct_command(instance, STACK_WRITE, 0, VCB_DONE_THRESH, 1, &uv_balance_threshold);
		      bq79600_tx(instance);
		      HAL_Delay(1 * n_devices);


													// and start OV UV comparators


		      buf = 0x5 ; //0x5;
		      bq79600_construct_command(instance, STACK_WRITE, 0, OVUV_CTRL, 1, &buf); // Set mode to run OV and UV round robin on all cells
		      bq79600_tx(instance);														// and start OV UV comparators


		      uint8_t ot_threshold = 0x12; // 0x3 = 80C threshold
		      bq79600_construct_command(instance, STACK_WRITE, 0, OTUT_THRESH, 1, &ot_threshold);
		      bq79600_tx(instance);

		      buf = 0x5 ; //0x5;
		      bq79600_construct_command(instance, STACK_WRITE, 0, OTUT_CTRL, 1, &buf); // Set mode to run OT and UT round robin on all cells
		      bq79600_tx(instance);

		      // setup ballancing
		        buf = 0x1;
			    bq79600_construct_command(instance, STACK_WRITE, 0, BAL_CTRL1, 1, &buf); // 5s ballancing timer
			    bq79600_tx(instance);

		        buf = 0x07; //0x07
		        bq79600_construct_command(instance, STACK_WRITE, 0, CB_CELL1_CTRL, 1 , &buf);
			    bq79600_tx(instance);
			    bq79600_construct_command(instance, STACK_WRITE, 0, CB_CELL2_CTRL, 1 , &buf);
			    bq79600_tx(instance);
			    bq79600_construct_command(instance, STACK_WRITE, 0, CB_CELL3_CTRL, 1 , &buf);
			    bq79600_tx(instance);
			    bq79600_construct_command(instance, STACK_WRITE, 0, CB_CELL4_CTRL, 1 , &buf);
			    bq79600_tx(instance);
			    bq79600_construct_command(instance, STACK_WRITE, 0, CB_CELL5_CTRL, 1 , &buf);
			    bq79600_tx(instance);
			    bq79600_construct_command(instance, STACK_WRITE, 0, CB_CELL6_CTRL, 1 , &buf);
			    bq79600_tx(instance);
			    bq79600_construct_command(instance, STACK_WRITE, 0, CB_CELL7_CTRL, 1 , &buf);
			    bq79600_tx(instance);
			    bq79600_construct_command(instance, STACK_WRITE, 0, CB_CELL8_CTRL, 1 , &buf);
			    bq79600_tx(instance);
			    bq79600_construct_command(instance, STACK_WRITE, 0, CB_CELL9_CTRL, 1 , &buf);
			    bq79600_tx(instance);
			    bq79600_construct_command(instance, STACK_WRITE, 0, CB_CELL10_CTRL, 1 , &buf);
			    bq79600_tx(instance);
			    bq79600_construct_command(instance, STACK_WRITE, 0, CB_CELL11_CTRL, 1 , &buf);
			    bq79600_tx(instance);
			    bq79600_construct_command(instance, STACK_WRITE, 0, CB_CELL12_CTRL, 1 , &buf);
			    bq79600_tx(instance);
			    bq79600_construct_command(instance, STACK_WRITE, 0, CB_CELL13_CTRL, 1 , &buf);
			    bq79600_tx(instance);
}

BQ_Data_Combined read_data(bq79600_t *instance , UART_HandleTypeDef  *uart_port , int n_devices,   int n_cells_per_device, int n_temp_pre_device)
{
	                     module_t modules[n_devices - 1] ;


	                     bq79600_construct_command(instance, STACK_READ, 0, DIETEMP1_HI, 2, NULL);
			 	         bq79600_tx(instance);
			 	         bq79600_bsp_ready(instance);
			 	         for (int i = 0; i < n_devices - 1; i++)
			 	         {
			 	           modules[i].dietemp = raw_to_float(instance->rx_buf[4 + i * 8]) * 0.025;
			 	         }


			 	         uint32_t start_vcells = VCELL1_HI - n_cells_per_device * 2 + 2;
			 	         bq79600_construct_command(instance, STACK_READ, 0, start_vcells, n_cells_per_device * 2, NULL);
			 	         bq79600_tx(instance);
			 	         bq79600_bsp_ready(instance);
			 	         for (int i = 0; i < n_devices - 1; i++)
			 	         {
			 	           for (int j = 0; j < n_cells_per_device; j++)
			 	           {
			 	             modules[i].vcells[j] =
			 	                 raw_to_float(instance->rx_buf[4 + i * (n_cells_per_device * 2 + 6) + 2 * j]) * 0.19073;
			 	           }
			 	         }



			 	         uint32_t start_temp = GPIO1_HI  ;
			 	         bq79600_construct_command(instance, STACK_READ, 0, start_temp, n_temp_pre_device * 2, NULL);
			 	         bq79600_tx(instance);
			 	         bq79600_bsp_ready(instance);

			 	         for (int i = 0; i < n_devices - 1; i++)
			 	         {
			 	           for (int j = 0; j < n_temp_pre_device; j++)
			 	           {
			 	        	   if(j==0)
			 	        	   {
			 	        		 modules[i].temperature[j] =voltage_to_temperature2(
			 	        		   	                 raw_to_float(&instance->rx_buf[4 + i * (n_temp_pre_device * 2 + 6) + 2 * j])  * 0.15259);
			 	        	   }
			 	        	   else
			 	        	   {
			 	             modules[i].temperature[j] =voltage_to_temperature(
			 	                 raw_to_float(&instance->rx_buf[4 + i * (n_temp_pre_device * 2 + 6) + 2 * j])  * 0.15259);
			 	        	   }
			 	           }
			 	         }




			 	         uint32_t start_temp_ref = TSREF_HI ;
			 	         bq79600_construct_command(instance, STACK_READ, 0, start_temp_ref, 2, NULL);
			 	         bq79600_tx(instance);
			 	         bq79600_bsp_ready(instance);
			 	         for (int i = 0; i < n_devices - 1; i++)
			 	             modules[i].t_ref =
			 	                 raw_to_float(&instance->rx_buf[4 + i * 8]) * 0.16954;



			 	         for (int i = 0; i < n_devices - 1; i++) modules[i].timestamp = HAL_GetTick();




			 	         bq79600_construct_command(instance, STACK_READ, 0, DEV_STAT, 1, NULL); // DEV_STAT READ.
			 	         bq79600_tx(instance);
			 	         bq79600_bsp_ready(instance);
			 	         for (int i = 0; i < n_devices - 1; i++)
			 	         modules[i].DEV_STAT_RAW = instance->rx_buf[4 + i * 7];



			 	         bq79600_construct_command(instance, STACK_READ, 0,  FAULT_UV1, 1, NULL); // DEV_STAT READ.
			 	         bq79600_tx(instance);
			 	         bq79600_bsp_ready(instance);
			 	         for (int i = 0; i < n_devices - 1; i++)
			 	         modules[i].UV_RAW_1 = instance->rx_buf[4 + i * 7];



			 	         bq79600_construct_command(instance, STACK_READ, 0,  FAULT_UV2, 1, NULL); // DEV_STAT READ.
			 	         bq79600_tx(instance);
			 	         bq79600_bsp_ready(instance);
			 	         for (int i = 0; i < n_devices - 1; i++)
			 	         modules[i].UV_RAW_2 = instance->rx_buf[4 + i * 7];


			 	         bq79600_construct_command(instance, STACK_READ, 0,  FAULT_OV1, 1, NULL); // DEV_STAT READ.
			 	         bq79600_tx(instance);
			 	         bq79600_bsp_ready(instance);
			 	         for (int i = 0; i < n_devices - 1; i++)
			 	         modules[i].OV_RAW_1 = instance->rx_buf[4 + i * 7];



			 	         bq79600_construct_command(instance, STACK_READ, 0,  FAULT_OV2, 1, NULL); // DEV_STAT READ.
			 	         bq79600_tx(instance);
			 	         bq79600_bsp_ready(instance);
			 	         for (int i = 0; i < n_devices - 1; i++)
			 	         modules[i].OV_RAW_2 = instance->rx_buf[4 + i * 7];

			 	         bq79600_construct_command(instance, STACK_READ, 0,  FAULT_UT, 1, NULL); // DEV_STAT READ.
			 	         bq79600_tx(instance);
			 	         bq79600_bsp_ready(instance);
			 	         for (int i = 0; i < n_devices - 1; i++)
			 	         modules[i].UT_RAW = instance->rx_buf[4 + i * 7];

			 	         bq79600_construct_command(instance, STACK_READ, 0,  FAULT_OT, 1, NULL); // DEV_STAT READ.
			 	         bq79600_tx(instance);
			 	         bq79600_bsp_ready(instance);
			 	         for (int i = 0; i < n_devices - 1; i++)
			 	         modules[i].OT_RAW = instance->rx_buf[4 + i * 7];




			 	         BQ_Data Data_to_send = {0} ;
			 	         BQ_Data_Combined Return_this_data;

			 	         // end of reading data from BQ79600
			 	         for (int i = 0; i < n_devices - 1; i++) // send data from bq to different tasks.
			 	         {

			 		     Data_to_send.BQ_Number = i ;
			 		     for (int j = 0; j < n_cells_per_device; j++)
			 		     {
			 		     Data_to_send.Bq_Voltages[j] = modules[i].vcells[j];

			 		     Data_to_send.Bq_Temperatures[j] = modules[i].temperature[j];
			 		     }
			 		     Data_to_send.dietemp =  modules[i].dietemp;
			 		     Data_to_send.Bq_Timestamp = modules[i].timestamp;
			 		     Data_to_send.T_ref = modules[i].t_ref;

			 		   //  uint8_t dev_stat =   bms_instance->rx_buf[4 + i * 7];
			 		     Data_to_send.Device_Stat.MAIN_ADC_RUN = (modules[i].DEV_STAT_RAW >> 0) & 0x01;
			 		     Data_to_send.Device_Stat.AUX_ADC_RUN = (modules[i].DEV_STAT_RAW >> 1) & 0x01;
			 		     Data_to_send.Device_Stat.CS_RUN = (modules[i].DEV_STAT_RAW >> 2) & 0x01;
			 		     Data_to_send.Device_Stat.OVUV_RUN = (modules[i].DEV_STAT_RAW >> 3) & 0x01;
			 		     Data_to_send.Device_Stat.OTUT_RUN = (modules[i].DEV_STAT_RAW >> 4) & 0x01;

			 		     for(int x = 0 ; x < 8  ; x++)
			 		     {
			 		    	 Data_to_send.UV_ERROR[x] = (modules[i].UV_RAW_2 >> x  ) & 0x01;
			 		    	 Data_to_send.OV_ERROR[x] = (modules[i].OV_RAW_2 >> x  ) & 0x01;
			 		    	 Data_to_send.OT_ERROR[x] = (modules[i].UT_RAW >> x  ) & 0x01;
			 		    	 Data_to_send.UT_ERROR[x] = (modules[i].OT_RAW >> x  ) & 0x01;


			 		     }
			 		     for(int x = 8 ; x <  n_cells_per_device ; x++)
			 		     {
			 		    	 Data_to_send.UV_ERROR[x] = (modules[i].UV_RAW_1 >> (x - 8 ) ) & 0x01;
			 		    	 Data_to_send.OV_ERROR[x] = (modules[i].OV_RAW_1 >> (x - 8 ) ) & 0x01;
			 		     }
			 		     // Reverse the array for consistancy
			 		     for(int x = 0; x < n_cells_per_device / 2; x++)
			 		     {
			 		         int tmp = Data_to_send.UV_ERROR[x];
			 		         int tmp2 = Data_to_send.OV_ERROR[x];

			 		         Data_to_send.UV_ERROR[x] = Data_to_send.UV_ERROR[n_cells_per_device - 1 - x];
			 		         Data_to_send.UV_ERROR[n_cells_per_device - 1 - x] = tmp;
			 		         Data_to_send.OV_ERROR[x] = Data_to_send.OV_ERROR[n_cells_per_device - 1 - x];
			 		         Data_to_send.OV_ERROR[n_cells_per_device - 1 - x] = tmp2;
			 		     }
			 		     for(int x = 0; x < n_temp_pre_device / 2; x++)
			 			 {
			 		         int tmp3 = Data_to_send.UT_ERROR[x];
			 		         int tmp4 = Data_to_send.OT_ERROR[x];
			 		         Data_to_send.UT_ERROR[x] = Data_to_send.UT_ERROR[n_temp_pre_device - 1 - x];
			 		         Data_to_send.UT_ERROR[n_cells_per_device - 1 - x] = tmp3;
			 		         Data_to_send.OT_ERROR[x] = Data_to_send.OT_ERROR[n_temp_pre_device - 1 - x];
			 		         Data_to_send.OT_ERROR[n_cells_per_device - 1 - x] = tmp4;
			 		     }
			 		    Return_this_data.Device[i] = Data_to_send;
			 	         }
			 	         return Return_this_data;

}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    static bq79600_t *instance = NULL;

    // Get instance (device 0)
    instance = open_bq79600_instance(0);
    if (instance == NULL)
        instance = open_bq79600_instance(0);

    if (instance != NULL) {
        instance->rx_len = size;
        bq79600_rx_callback(instance);
        HAL_UARTEx_ReceiveToIdle_IT(&huart4, instance->rx_buf, sizeof(instance->rx_buf));
    }
}

// USB RX callback


// Convert voltage (mV) to temperature (°C) using a log fit
int voltage_to_temperature(float voltage)
{
    if (voltage > 3500)
        return 0;
    else
        return (int)(-50.2f * logf(voltage) + 416.0f);
}

// Convert voltage (mV) to temperature (°C) for 0603 NTC using sigmoid fit
int voltage_to_temperature2(float voltage)
{
    if (voltage > 3500)
        return 0;
    else
        return (int)(154.0f / (1.0f + expf(0.0012f * (voltage - 910.0f))));
}
