#pragma once

#include "array"

#include "tx_api.h"
#include "main.h"
#include "usart.h"

#include "utils.hpp"
#include "config.hpp"

namespace PUTM
{
    namespace Bq796xx
    {
        template<size_t STACK_SIZE>
        class Device
        {
        private: 
            /* Bq79600 has a preset baud rate of 1Mbps [bps] */
            constexpr static inline uint32_t default_baudrate = 1'000'000;
            /* Time hold wakeup, in range <2500, 3000> [us] */
            constexpr static inline uint32_t t_wakeup = 2'600;
            /* Rx timeout [us] (2 * defualt) */
            constexpr static inline uint32_t t_rx_timeout = 500;
            /* Baudrate for init [bps], assume 1 bit high before, 6 bits of low time and 1 bit high after, this should create the required pattern */
            constexpr static inline uint32_t baudrate_wakeup = (uint32_t)(1'000'000.0 / (double)t_wakeup * 6.0);
            /* Rx timeout represented in baudblocks */
            constexpr static inline uint32_t rx_timeout_baudblocks = (uint32_t)((double)t_rx_timeout * (double)default_baudrate / 1'000'000.0);
            /* uV per bit */
            constexpr static inline double v_lsb_adc = 190.73e-6;

            /* stm32 mutex handle */
            UART_HandleTypeDef *huart;

            /* mutex used for interupt synchronisation of write/read */
            TX_SEMAPHORE semaphore;

            /* -- */
            size_t read_size;

            /* for now leave the size at 256 */
            std::array<uint8_t, 256> out { 0 };
            /* for now leave the size at 256 */
            std::array<uint8_t, 256> in { 0 };
        public:
            /**
            *	@brief struct for local stack status storage
            */
            struct StackDeviceStatus
            {
                bool ovuv[16] { false };
                bool otut[8] { false };
            };
            StackDeviceStatus stack_device_status[STACK_SIZE];
        public:
            /**
            *	@brief struct for local stack data storage
            */
            struct StackDeviceData
            {
                float voltages[16] { 0 };
                float temperatures[8] { 0 };
            };
            StackDeviceData stack_device_data[STACK_SIZE] { };
        private:
            enum struct ReqType : uint8_t
            {
                Single 		= 0b00,
                Stack		= 0b01,
                Broadcast	= 0b10
            };
        private:
            /**
            * 	@brief 	Staticly calculate the number of devices being read from
            *	@return	Number of devices being read from
            */
        template<ReqType REQ_TYPE>
        static consteval size_t read_count()
        {
            switch (REQ_TYPE)
            {
            case ReqType::Single:
                return 1;
            case ReqType::Stack:
                return STACK_SIZE;
            case ReqType::Broadcast:
                return STACK_SIZE + 1;
            }
            Utils::throw_consteval_failure("WRONG");
        }
        private:
            template<ReqType REQ_TYPE>
            uint8_t init_byte_write(uint8_t data_size);
        private:
            template<ReqType REQ_TYPE>
            uint8_t init_byte_read();
        public:
            /**
            *	@brief Init Bq796xx, set receiver timeout to t_rx_timeout (~300 us) for the uart handler to for data in IT or DMA mode
            *	@param `huart` uart handle
            */
            Device(UART_HandleTypeDef *huart);
        public:
            HAL_StatusTypeDef init();
        public:
            /**
            * 	@brief 	This function inits other uart communication, call this function firsts
            * 	@retval	HAL_OK
            */
            HAL_StatusTypeDef init_uart();
        public:
            /**
            * 	@brief 	This function inits whole stack communication
            * 	@retval	HAL_OK
            */
            HAL_StatusTypeDef init_stack();
        public:
            /**
            *   @brief  This function inits voltage measurement
            * 	@retval	HAL_OK
            */
            HAL_StatusTypeDef init_voltage_measurement();
        public:
            /** 	
            *	@brief	Init undervoltage and overvoltage protection
            *	@param	`undervoltage` in mV, must be between 1200 and 3100
            *	@param	`overvoltage` in mV, must be between 4175 and 4475
            *	@retval HAL_OK when done, HAL_BUSY when init in progress, HAL_ERROR on fail
            */
            HAL_StatusTypeDef init_ovuv(uint32_t undervoltage, uint32_t overvoltage);
        public:
            /** 	
            *	@brief	Disable undervoltage detection on selected channels, every call overrides past calls
            *	@param 	channels array with channel numbers
            *	@param	size size of array
            *	@return HAL_OK when done, HAL_BUSY when init in progress, HAL_ERROR on fail
            */
            // HAL_StatusTypeDef set_uv_disable(Utils::Channel *channels, size_t size);
        public:
            /**
             *  @brief 	This function inits temperature measurement, this function sets GPIO pins to ADC_OTUT mode
             *  @return HAL_OK
             */
            HAL_StatusTypeDef init_temperature_measurements();
        public:
            /**
             *  @brief 	This function inits otut thresholds
             *  @param  undertemperature threshold represendeted as a percentage 66% to 80% in steps of 2!
             *  @param  overtemperature threshold represendeted as a percentage 10% to 39% in steps of 1
             *  @return HAL_OK
             */
            HAL_StatusTypeDef init_otut(uint8_t undertemperature, uint8_t overtemperature);
        public:
            /**
             *  @brief  This function starts main adc conversion
             *  @return HAL_OK
             */
            HAL_StatusTypeDef start_measurements();
        public:
            /**
            *	@brief Poll stack status to local storage
             *  @return HAL_OK
            */
            HAL_StatusTypeDef update_status();
        public:
            /**
            *	@brief Poll stack data to local storage
             *  @return HAL_OK
            */
            HAL_StatusTypeDef update_data();
        public:
            /**
            * 	@brief 	Wake up function for BQ79600 IC, this functions tries to hold the MOSI line
            *			for aprox ~2.5ms
            * 	@retval	HAL_BUSY when wakeing up is in progress, HAL_OK when done
            */
            HAL_StatusTypeDef wake_up();
        public:
            /**
            * 	@brief 	Send `data` of `size` to a device at `address` in `REQ_TYPE` mode. All registers are 1 byte in len, 
            *			so any write with size > 1 is automaticaly interpreted as serial write begining at address `reg_address`.
            *			If data was send before this function can be called without any parameters to check the `writeSingle` state: 
            *			`HAL_BUSY` (sending) or `HAL_OK` (done)
            *	@tparam	`REQ_TYPE` write type
            * 	@param 	`data` data, cant be largen than 8 bytes
            *	@param	`size` size of data cant be larger than 8
            *	@param 	`address` address of a device to be written to - assumes 0
            * 	@retval	HAL_BUSY when writeSingle is in progress, HAL_OK when done
            */
            template<ReqType REQ_TYPE>
            HAL_StatusTypeDef write(uint8_t *data, size_t size, uint16_t reg_address, uint8_t address = 0);
        public:
            // FIXME: this code is kinda ass, change it in the future to be more predictable maybe just add size?
            /**
            * 	@brief 	Read `data` of `size` from a device at `address` in `REQ_TYPE` mode. All registers are 1 byte in len, 
            *			so any read with size > 1 is automaticaly interpreted as serial read begining at address `reg_address`.
            *			This function blocks it's caller however it will return HAL_BYSY, when reading data is in proggress 
            *			so it's up to the user to hadle it properly. This function times out after 1ms.
            *	@tparam	REQ_TYPE read type, Single, Stack or Broadcast
            * 	@param 	data copies the received data to provided container, when new data was received. If for any reason data received
            *			was coruppted or not received it will not be coppied over to the procided buffer. Data should point to a buffer of an
            *			appropriate size - size for single read, size * STACK_SIZE for stack read
            *	@param 	count number of registers to read not the size of the array!
            *	@param 	address address of a device to be written to in signle mode, assumes 0. In other modes it is ignored
            * 	@retval	HAL_BUSY when read is in progress, HAL_OK when done or caller provided no data/size, HAL_TIMEOUT when read operation wasn't
            *			properly executed.
            */
            template<ReqType REQ_TYPE>
            HAL_StatusTypeDef read(uint8_t *data, size_t count, uint16_t reg_address, uint8_t address = 0);
        };   
    }
}

template class PUTM::Bq796xx::Device<PUTM::Config::STACK_SIZE>;