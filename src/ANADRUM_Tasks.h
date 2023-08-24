#pragma once

#define LOOP_TASK_CORE 1

// +-----------------------------------------------------------------------+
//                                                      LED Task
extern TaskHandle_t led_task_handle;
#define LED_TASK_PRIORITY 1
#define LED_TASK_CORE LOOP_TASK_CORE
#define LED_TASK_DELAY 40
#define LED_TASK_STACK_SIZE 12000


// +-----------------------------------------------------------------------+
//                                                      LED Task
extern TaskHandle_t channel__task_handle;
#define CHANNEL__TASK_PRIORITY 0
#define CHANNEL__TASK_CORE 0
#define CHANNEL__TASK_DELAY 2
#define CHANNEL__TASK_STACK_SIZE 12000
