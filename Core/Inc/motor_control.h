#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

void Motor_open_recir();
void Motor_stop_recir();
void Motor_closed_recir();

void Motor_open_chest();
void Motor_stop_chest();
void Motor_closed_chest();

void Motor_open_window();
void Motor_stop_window();
void Motor_closed_window();


void Motor_open_legs();
void Motor_stop_legs();
void Motor_closed_legs();


#ifdef __cplusplus
}
#endif

#endif 