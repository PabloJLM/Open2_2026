#pragma once
#include "packet.h"

// Inicializa la MPU6050. Devuelve false si no responde.
bool imu_init();

// Lee una muestra cruda en s. Debe tardar bastante menos que 7.8 ms.
void imu_read(imu_sample_t *s);
