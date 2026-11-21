#pragma once
// Contrato de datos entre nodos, gateway y el registrador en Python.
// Si cambias algo aqui, cambia tambien tools/logger.py y corre tools/sync.sh

#include <stdint.h>
#include <stddef.h>

#define FS_HZ      128   // frecuencia de muestreo del paper
#define N_SAMP     8     // muestras por paquete, 62.5 ms
#define PKT_MAGIC  0xB7  // filtra paquetes ESP-NOW de otros equipos
#define FR_SYNC0   0xA5  // marco serial gateway a PC
#define FR_SYNC1   0x5A

// Muestra cruda de la MPU6050, sin convertir a unidades fisicas.
typedef struct __attribute__((packed)) {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} imu_sample_t;

// Paquete ESP-NOW de un nodo.
// t0_us es el instante del tick de la primera muestra, reloj del nodo.
// n_miss cuenta ticks perdidos por el nodo dentro del paquete.
typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint8_t  node_id;
  uint8_t  n_miss;
  uint8_t  rsv;
  uint32_t seq;
  uint32_t t0_us;
  imu_sample_t s[N_SAMP];
} imu_pkt_t;

// Elemento que el gateway guarda en cola y manda a la PC.
typedef struct __attribute__((packed)) {
  uint32_t rx_us;     // reloj del gateway al recibir
  imu_pkt_t pkt;
} rx_item_t;

// Marco serial hacia la PC
//   FR_SYNC0 FR_SYNC1 len(1 byte) rx_item_t crc8
// len es sizeof(rx_item_t), crc8 se calcula sobre rx_item_t.
static inline uint8_t crc8(const uint8_t *d, size_t n)
{
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++) {
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
  }
  return c;
}

#ifdef __cplusplus
static_assert(sizeof(imu_sample_t) == 12, "imu_sample_t debe medir 12 bytes");
static_assert(sizeof(imu_pkt_t) == 108, "imu_pkt_t debe medir 108 bytes");
static_assert(sizeof(rx_item_t) == 112, "rx_item_t debe medir 112 bytes");
#else
_Static_assert(sizeof(imu_pkt_t) == 108, "imu_pkt_t debe medir 108 bytes");
_Static_assert(sizeof(rx_item_t) == 112, "rx_item_t debe medir 112 bytes");
#endif
