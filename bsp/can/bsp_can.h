#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"

// typedef struct 
// {
//     CAN_HandleTypeDef* can_handle;
//     uint32_t tx_id;
//     uint32_t rx_id;
//     void (*can_module_callback)(can_instance*);
//     void* id;

// } can_instance_config;

typedef struct
{
    CAN_HandleTypeDef* can_handle;
    int master_id;
    int slave_id;
  
}CAN_decive_parameter_t; 

void can_filter_init(void);

#endif
