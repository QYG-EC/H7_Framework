#ifndef TEST_FDCAN_H
#define TEST_FDCAN_H
#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif
    typedef struct { uint32_t PSR, CCCR; } FDCAN_GlobalTypeDef;
#define FDCAN_PSR_BO 0x80U
#define FDCAN_CCCR_INIT 1U
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
    typedef struct
    {
        unsigned bus;
        FDCAN_GlobalTypeDef *Instance;
    } FDCAN_HandleTypeDef;
    extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
    typedef enum
    {
        HAL_OK,
        HAL_ERROR
    } HAL_StatusTypeDef;
    typedef struct
    {
        uint32_t IdType, FilterIndex, FilterType, FilterConfig, FilterID1, FilterID2;
    } FDCAN_FilterTypeDef;
    typedef struct
    {
        uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator;
        uint32_t BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
    } FDCAN_TxHeaderTypeDef;
    typedef struct
    {
        uint32_t Identifier, IdType, RxFrameType, DataLength, FDFormat;
    } FDCAN_RxHeaderTypeDef;
#define FDCAN_STANDARD_ID 0U
#define FDCAN_EXTENDED_ID 0x40000000U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_REMOTE_FRAME 0x20000000U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_FD_CAN 0x00200000U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_OFF 0U
#define FDCAN_NO_TX_EVENTS 0U
#define FDCAN_FILTER_MASK 2U
#define FDCAN_FILTER_TO_RXFIFO0 1U
#define FDCAN_ACCEPT_IN_RX_FIFO0 0U
#define FDCAN_REJECT 2U
#define FDCAN_REJECT_REMOTE 1U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
#define FDCAN_RX_FIFO0 0x40U
    uint32_t __get_PRIMASK(void);
    void __disable_irq(void);
    void __enable_irq(void);
    void __set_PRIMASK(uint32_t);
    void __DMB(void);
    void Error_Handler(void);
    HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef*, FDCAN_FilterTypeDef*);
    HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef*, uint32_t, uint32_t, uint32_t, uint32_t);
    HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef*, uint32_t, uint32_t);
    HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef*);
    uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef*, uint32_t);
    HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef*, uint32_t, FDCAN_RxHeaderTypeDef*, uint8_t*);
    uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef*);
    HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef*, const FDCAN_TxHeaderTypeDef*, const uint8_t*);
    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef*, uint32_t);
#ifdef __cplusplus
}
#endif
#endif
