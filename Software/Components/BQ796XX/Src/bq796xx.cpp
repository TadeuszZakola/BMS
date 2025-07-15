#include "tx_api.h"

#include "bq796xx.hpp"
#include "bq796xx/regs.hpp"
#include "bq796xx/crc16ibm.hpp"
#include "utils.hpp"
#include "wrapper/uart.hpp"
#include "algorithm"

using namespace PUTM;
using namespace Utils;
using namespace PUTM::Bq796xx;
using namespace PUTM::Bq796xx::Regs;
using namespace PUTM::Bq796xx::Types;


template<size_t STACK_SIZE>
Device<STACK_SIZE>::Device(UART_HandleTypeDef *huart) : huart(huart) { }

template<size_t STACK_SIZE>
template<Device<STACK_SIZE>::ReqType REQ_TYPE>
uint8_t Device<STACK_SIZE>::init_byte_write(uint8_t data_size)
{
    // uint8_t rsvd = 0b0000'0000;
    uint8_t frame_type = 0b1'0000000;
    uint8_t req_type = 0b0'001'0000 | (uint8_t)REQ_TYPE << 5;
    if(1 > data_size or data_size > 8) Error_Handler();
    data_size--;

    return frame_type | req_type | data_size;
}

template<size_t STACK_SIZE>
template<Device<STACK_SIZE>::ReqType REQ_TYPE>
uint8_t Device<STACK_SIZE>::init_byte_read()
{
    // uint8_t rsvd = 0b0000'0000;
    uint8_t frame_type = 0b1'0000000;
    uint8_t req_type = 0b0'000'0000 | (uint8_t)REQ_TYPE << 5;
    // uint8_t data_size = 0b0000'0000;

    return frame_type | req_type;
}

template<size_t STACK_SIZE>
HAL_StatusTypeDef Device<STACK_SIZE>::init()
{
    tx_semaphore_create(&semaphore, "semaphore", 0);

    return HAL_OK;
}

template<size_t STACK_SIZE>
HAL_StatusTypeDef Device<STACK_SIZE>::init_uart()
{
    HAL_UART_ReceiverTimeout_Config(huart, rx_timeout_baudblocks);
    HAL_UART_EnableReceiverTimeout(huart);
    //SET_BIT(huart->Instance->CR1, USART_CR1_RTOIE);

    return HAL_OK;
}

template<size_t STACK_SIZE>
HAL_StatusTypeDef Device<STACK_SIZE>::init_stack()
{
    uint8_t data[16];

    wake_up();

    /* wait at least 4ms */
    tx_thread_sleep(4);

    /* cmd wake up slaves */
    data[0] = convert_to<uint8_t>((Control1){.send_wake = true});
    write<ReqType::Single>(data, 1, address_of<Control1>());

    /* wait ~15ms */
    tx_thread_sleep(15);

    /* dummy write 0x00, sync internal dlls */
    data[0] = 0x00;
    for(size_t step = 0; step < 8; step++)
    {
        write<ReqType::Broadcast>(data, 1, 0x343 + step);
    }

    /* enable auto adressing */
    data[0] = 0x01;
    write<ReqType::Broadcast>(data, 1, 0x309);

    /* auto addressing */
    for(size_t address = 0; address <= STACK_SIZE; address++)
    {
        data[0] = address;
        write<ReqType::Broadcast>(data, 1, 0x306);
    }

    /* set bq7961x as stack device */
    data[0] = 0x02;
    write<ReqType::Broadcast>(data, 1, 0x308);

    /* set which bq is last */
    data[0] = 0x03;
    write<ReqType::Single>(data, 1, 0x308, STACK_SIZE);

    /* dummy read sync internal dlls */
    for(size_t step = 0; step < 8; step++)
    {
        read<ReqType::Stack>(nullptr, 1, 0x343 + step);
    }

    /* verify adresses */
    
    // TODO: internal status error
    read<ReqType::Stack>(data, 1, 0x306);

    return HAL_OK;
}

template<size_t STACK_SIZE>
HAL_StatusTypeDef Device<STACK_SIZE>::init_voltage_measurement()
{
    uint8_t data[16];

    /* TODO: do it properly >.> */
    /* set active cells in series */
    data[0] = 0x8; 
    write<ReqType::Stack>(data, 1, 0x0003);

    return HAL_OK;
}

template<size_t STACK_SIZE>
HAL_StatusTypeDef Device<STACK_SIZE>::init_ovuv(uint32_t undervoltage, uint32_t overvoltage)
{
    uint8_t data[2] = { 0 };

    /* clamp */
    undervoltage = std::clamp(undervoltage, 1200ul, 3100ul);
    // FIXME: fuuuuuck
    /* this one is complicated... for now leave it like that */
    overvoltage = std::clamp(overvoltage, 4175ul, 4475ul);

    /* set boundries */
    data[0] = (uint8_t)(((undervoltage - 1200) / 50) & 0x3f);
    // FIXME: fuuuuuck
    /* this one is complicated... for now leave it like that */
    data[1] = (uint8_t)((((overvoltage - 4175) / 25) + 0x22) & 0x3f);

    /* write to ov uv threshold registers */
    write<ReqType::Stack>(data, 2, 0x0009);

    /* enable ovuv */
    data[0] = convert_to<uint8_t>((OVUVCtrl){.ovuv_mode = ScanMode::RoundRobin, .ovuv_go = true});

    /* send twice, bq requires another 'go' cmd when setting are changed */
    write<ReqType::Stack>(data, 1, address_of<OVUVCtrl>());
    write<ReqType::Stack>(data, 1, address_of<OVUVCtrl>());

    return HAL_OK;
}

template<size_t STACK_SIZE>
HAL_StatusTypeDef Device<STACK_SIZE>::init_temperature_measurements()
{
    uint8_t data[8] { 0 };

    data[0] = convert_to<uint8_t>((GPIOConf1){ .gpio1 = GpioMode::AdcOtut, .gpio2 = GpioMode::AdcOtut });
    data[1] = convert_to<uint8_t>((GPIOConf2){ .gpio3 = GpioMode::AdcOtut, .gpio4 = GpioMode::AdcOtut });
    data[2] = convert_to<uint8_t>((GPIOConf3){ .gpio5 = GpioMode::AdcOtut, .gpio6 = GpioMode::AdcOtut });
    data[3] = convert_to<uint8_t>((GPIOConf4){ .gpio7 = GpioMode::AdcOtut, .gpio8 = GpioMode::AdcOtut });
    //
    write<ReqType::Stack>(data, 4, address_of<GPIOConf1>());
    
    return HAL_OK;
}

template<size_t STACK_SIZE>
HAL_StatusTypeDef Device<STACK_SIZE>::init_otut(uint8_t undertemperature, uint8_t overtemperature)
{
    uint8_t data[1] = { 0 };

    /* set boundries */
    undertemperature = std::clamp(undertemperature, 66ui8, 80ui8);
    overtemperature = std::clamp(overtemperature, 10ui8, 39ui8);

    undertemperature = (uint8_t)((undertemperature - 66 / 2) & 0x08);
    overtemperature = (uint8_t)((overtemperature - 10) & 0x1f);
    
    data[0] = convert_to<uint8_t>((OTUTThresh){ .ot_thr = overtemperature, .ut_thr = undertemperature });

    /* write to ov uv threshold registers */
    write<ReqType::Stack>(data, 1, address_of<OTUTThresh>());

    /* enable otut */
    data[0] = convert_to<uint8_t>((OVUVCtrl){ .ovuv_mode = ScanMode::RoundRobin, .ovuv_go = true });

    /* send twice, bq requires another 'go' cmd when setting are changed */
    write<ReqType::Stack>(data, 1, 0x032C);
    write<ReqType::Stack>(data, 1, 0x032C);

    return HAL_OK;
}

template<size_t STACK_SIZE>
HAL_StatusTypeDef Device<STACK_SIZE>::start_measurements()
{
    uint8_t data[1] { 0 };
    /* set adc continous, start conversion, enable lpf */
    data[0] = convert_to<uint8_t>((AdcCtrl1){ .main_mode = ScanMode::RoundRobin, .main_go = true, .lpf_cell_en = true }); 
    write<ReqType::Stack>(data, 1, address_of<AdcCtrl1>());

    return HAL_OK;
}

// template<size_t STACK_SIZE>
// HAL_StatusTypeDef Device<STACK_SIZE>::set_uv_disable(Utils::Channel *channels, size_t size)
// {
//     uint16_t buffer;
//     for(size_t i = 0; i < size; i++)
//     {
//         buffer |= (1 << (uint8_t)channels[i]);
//     }
//
//     uint8_t data[] = { (uint8_t)(buffer >> 8 & 0xff), (uint8_t)(buffer & 0xff) };
//    
//     write<ReqType::Stack>(data, 2, sta<UVDisable1>());
//
//     return HAL_OK;
// }

template<size_t STACK_SIZE>
HAL_StatusTypeDef Device<STACK_SIZE>::update_status()
{
    using namespace Utils;

    constexpr size_t ovuv_size = 4;
    constexpr size_t otut_size = 2;

    uint8_t buffer[(ovuv_size + otut_size) * STACK_SIZE] { 0 };

    /* read ov1/2, uv1/2, ot and ut, 0x053C, address of FAULT_OV1, yes i started getting lazy */
    read<ReqType::Stack>(buffer, (ovuv_size + otut_size), 0x053C);
    
    for(size_t idev = 0; idev < STACK_SIZE; idev++)
    {
        size_t offset = (ovuv_size + otut_size) * idev;
        /* over and under voltage */
        uint16_t ov_tmp = (uint16_t)buffer[0 + offset] << 8 | (uint16_t)buffer[1 + offset];
        uint16_t uv_tmp = (uint16_t)buffer[2 + offset] << 8 | (uint16_t)buffer[3 + offset];
        uint16_t ovuv_tmp = ov_tmp | uv_tmp;

        /* over and under temperature */
        uint8_t ot_tmp = (uint16_t)buffer[4 + offset];
        uint8_t ut_tmp = (uint16_t)buffer[5 + offset];
        uint8_t otut_tmp = ot_tmp | ut_tmp;

        /* voltages status */
        for(size_t ich = 0; ich < 16; ich++)
        {
            size_t bit_index = 1 << ich;
            stack_device_status[idev].ovuv[ich] = (bool)(ovuv_tmp & bit_index);
        }

        /* temperatures status */
        for(size_t igio = 0; igio < 8; igio++)
        {
            size_t bit_index = 1 << igio;
            stack_device_status[idev].otut[igio] = (bool)(otut_tmp & bit_index);
        }
    }

    return HAL_OK;
}

template<size_t STACK_SIZE>
HAL_StatusTypeDef Device<STACK_SIZE>::update_data()
{
    using namespace Utils;

    /* 16 cells * 2 bytes */
    constexpr size_t data_count = 16 * 2;
    uint8_t buffer[data_count * STACK_SIZE] { 0 };

    /* read voltages, address of VCELL16_HI */
    read<ReqType::Stack>(buffer, data_count, 0x0568);

    /* voltages */
    for(size_t idev = 0; idev < STACK_SIZE; idev++)
    {
        for(size_t ich = 0; ich < 16; ich++)
        {
            size_t index = idev * data_count + ich * 2;
            int16_t volt = ((uint16_t)(buffer[index]) << 8 | (uint16_t)(buffer[index + 1]));
            
            stack_device_data[idev].voltages[15 - ich] = -(~volt + 1) * v_lsb_adc;
        }
    }

    /* read temperatures, address of GPIO1_HI */
    read<ReqType::Stack>(buffer, data_count, 0x058E);

    /* temperatures */
    for(size_t idev = 0; idev < STACK_SIZE; idev++)
    {
        for(size_t igio = 0; igio < 8; igio++)
        {
            size_t index = idev * data_count + igio * 2;
            int16_t volt = ((uint16_t)(buffer[index]) << 8 | (uint16_t)(buffer[index + 1]));
            
            stack_device_data[idev].temperatures[igio] = -(~volt + 1) * v_lsb_adc;
        }
    }
    
    return HAL_OK;
}

template<size_t STACK_SIZE>
HAL_StatusTypeDef Device<STACK_SIZE>::wake_up()
{
    /* prevent override during checks */
    volatile UartState state (huart->gState);

    if(not state.init_done or state.status == UartStatus::Error) Error_Handler();

    /* uart sends lsb first, this sequence includes start bit for a total of '6' bits */
    out.at(0) = 0b1110'0000;

    /* init uart with a custom baudrate which allows for generating wakeup signal */
    if(HAL_UART_DeInit(huart) != HAL_OK) Error_Handler();
    huart->Init.BaudRate = baudrate_wakeup;
    if(HAL_UART_Init(huart) != HAL_OK) Error_Handler();

    pUART_CallbackTypeDef callback_write = [](UART_HandleTypeDef* huart)
    {
        if(huart->UserData == nullptr) Error_Handler();
        Device *bq = (Device*)huart->UserData;

        volatile UartState state (huart->gState);

        if(not state.init_done or state.status == UartStatus::Error) Error_Handler();

        if(HAL_UART_DeInit(huart) != HAL_OK) Error_Handler();
        huart->Init.BaudRate = default_baudrate;
        if(HAL_UART_Init(huart) != HAL_OK) Error_Handler();

        huart->TxCpltCallback = HAL_UART_TxCpltCallback;

        tx_semaphore_put(&bq->semaphore);
    };

    huart->TxCpltCallback = callback_write;
    huart->UserData = (void*)this;

    if(HAL_UART_Transmit_DMA(huart, (uint8_t*)out.begin(), 1) != HAL_OK) Error_Handler();
    auto notify_received = tx_semaphore_get(&semaphore, 10);

    if(notify_received != 0) Error_Handler();

    return HAL_OK;
}

template<size_t STACK_SIZE>
template<Device<STACK_SIZE>::ReqType REQ_TYPE>
HAL_StatusTypeDef Device<STACK_SIZE>::write(uint8_t *data, size_t size, uint16_t reg_address, uint8_t address)
{
    volatile UartState state (huart->gState);

    if(not state.init_done or state.status == UartStatus::Error) Error_Handler();
    
    size_t i = 0;
    out.at(i++) = init_byte_write<REQ_TYPE>(size);

    if constexpr(REQ_TYPE == ReqType::Single) out.at(i++) = address;

    out.at(i++) = (uint8_t)(reg_address >> 8);
    out.at(i++) = (uint8_t)(reg_address);

    std::copy(data, data + size, out.begin() + i);

    uint16_t crc = crc16.fast(out.begin(), size + i);
    out.at(size + i++) = (uint8_t)(crc >> 8);
    out.at(size + i++) = (uint8_t)(crc);

    pUART_CallbackTypeDef callback_write = [](UART_HandleTypeDef* huart)
    {
        if(huart->UserData == nullptr) Error_Handler();
        Device *bq = (Device*)huart->UserData;

        volatile UartState state (huart->gState);
        if(not state.init_done or state.status == UartStatus::Error) Error_Handler();

        huart->TxCpltCallback = HAL_UART_TxCpltCallback;
        huart->UserData = nullptr;

        tx_semaphore_put(&bq->semaphore);
    };

    huart->UserData = (void*)this;
    huart->TxCpltCallback = callback_write;

    if(HAL_UART_Transmit_DMA(huart, (uint8_t*)out.begin(), size + i) != HAL_OK) Error_Handler();
    auto notify_received = tx_semaphore_get(&semaphore, 10);

    if(notify_received != 0) Error_Handler();
    return HAL_OK;
}

template<size_t STACK_SIZE>
template<Device<STACK_SIZE>::ReqType REQ_TYPE>
HAL_StatusTypeDef Device<STACK_SIZE>::read(uint8_t *data, size_t count, uint16_t reg_address, uint8_t address)
{

    /* prevent override during checks? */
    volatile UartState state (huart->gState);

    std::fill(in.begin(), in.end(), 0);
    
    size_t i = 0;
    out.at(i++) = init_byte_read<REQ_TYPE>();

    if constexpr(REQ_TYPE == ReqType::Single) out.at(i++) = address;

    out.at(i++) = (uint8_t)(reg_address >> 8);
    out.at(i++) = (uint8_t)(reg_address);

    out.at(i++) = count - 1;

    uint16_t crc = crc16.fast(out.begin(), i);
    out.at(i++) = (uint8_t)(crc >> 8);
    out.at(i++) = (uint8_t)(crc);

    pUART_CallbackTypeDef callback_write = [](UART_HandleTypeDef* huart)
    {
        if(huart->UserData == nullptr) Error_Handler();
        Device *bq = (Device*)huart->UserData;
        
        volatile UartState state (huart->gState);
        if(not state.init_done or state.status == UartStatus::Error) Error_Handler();

        HAL_UART_EnableReceiverTimeout(huart);
        if(HAL_UART_Receive_DMA(huart, (uint8_t*)bq->in.begin(), bq->read_size) != HAL_OK) Error_Handler();
        
        huart->TxCpltCallback = HAL_UART_TxCpltCallback;
    };

    pUART_CallbackTypeDef callback_read = [](UART_HandleTypeDef* huart)
    {
        if(huart->UserData == nullptr) Error_Handler();
        Device *bq = (Device*)huart->UserData;

        volatile UartState state (huart->gState);
        if(not state.init_done or state.status == UartStatus::Error) Error_Handler();

        huart->RxCpltCallback = HAL_UART_RxCpltCallback;
        huart->UserData = nullptr;

        tx_semaphore_put(&bq->semaphore);
    };

    read_size = (count + 6) * read_count<REQ_TYPE>();
    huart->TxCpltCallback = callback_write;
    huart->RxCpltCallback = callback_read;
    huart->UserData = (void*)this;

    if(HAL_UART_Transmit_DMA(huart, (uint8_t*)out.begin(), i) != HAL_OK) Error_Handler();
    auto notify_received = tx_semaphore_get(&semaphore, 10);

    if(notify_received != 0) 
    { 
        HAL_UART_Abort(huart); 
        return HAL_ERROR; 
    }
    
    if(data == nullptr) 
    {
        return HAL_ERROR;
    }

    constexpr size_t size = read_count<REQ_TYPE>();
    auto it_data_begin = in.begin() + 4;
    auto it_data_end = in.begin() + 4 + count;
    auto it_data = data;
    
    for(size_t i = 0; i < size; i++)
    {
        // TODO: CRC?
        std::copy(it_data_begin, it_data_end, it_data);
        it_data_begin += count + 6;
        it_data_end += count + 6;
        it_data += count;
    }

    return HAL_OK; 
}