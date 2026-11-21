#include "imu.h"
#include "config.h"

// STUB. Aqui va tu lectura de la MPU6050 por I2C con el driver de IDF.
// Mientras tanto genera una rampa para probar el enlace y las perdidas.

static int16_t k = 0;

bool imu_init()
{
  // TODO i2c master, despertar la MPU, rango del acelerometro, filtro interno
  return true;
}

void imu_read(imu_sample_t *s)
{
  // TODO reemplazar por la lectura real, en orden ax ay az gx gy gz
  s->ax = k;
  s->ay = (int16_t)(-k);
  s->az = 16384;
  s->gx = NODE_ID;
  s->gy = NODE_ID;
  s->gz = NODE_ID;
  k++;
}
