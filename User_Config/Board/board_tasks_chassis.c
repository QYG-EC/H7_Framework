#include "board_tasks.h"
#include "main.h"

void Board_CreateTasks(void)
{
    CanTxTaskHandle = osThreadNew(Can_Tx_Task, NULL, &CanTxTask_attributes);
    StatusTaskHandle = osThreadNew(Status_Task, NULL, &StatusTask_attributes);
    ControlTaskHandle = osThreadNew(Control_Task, NULL, &ControlTask_attributes);
    if (CanTxTaskHandle == NULL ||
        StatusTaskHandle == NULL ||
        ControlTaskHandle == NULL)
    {
        Error_Handler();
    }
}
